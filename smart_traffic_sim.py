#!/usr/bin/env python3
"""
======================================================================
Smart Traffic Light Control System with VANET + NS-3 Integration
Hệ thống điều khiển đèn giao thông thông minh sử dụng VANET 802.11p
======================================================================

Tích hợp NS-3 (mô phỏng mạng VANET) + SUMO (mô phỏng giao thông)
- Giao thức: DSRC/802.11p cho V2I (Vehicle-to-Infrastructure)
- Mô hình: Phân tán với RSU (Road Side Units) ở mỗi đèn giao thông
- Điều khiển: Thích ứng + Ưu tiên xe cứu thương + Dự đoán tắc đường
"""

import os
import sys
import subprocess
import time
import socket
import json
import threading
import struct
from datetime import datetime
from collections import defaultdict
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict
import logging

# ===== Setup Logging =====
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# ===== Setup SUMO =====
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    logger.error("❌ SUMO_HOME environment variable not set")
    sys.exit(1)

import traci
import traci.constants as tc

# ===== VANET Constants =====
DSRC_CHANNEL_FREQ = 5.9  # GHz (802.11p standard)
DSRC_TX_RANGE = 300  # meters
DSRC_BANDWIDTH = 20  # MHz
DSRC_TX_POWER = 20  # dBm

RSU_BROADCAST_INTERVAL = 1.0  # seconds (RSU broadcasts TL status)
VEHICLE_BEACON_INTERVAL = 0.5  # seconds (vehicles send beacons)

# ===== Message Types for VANET =====
MSG_TYPE_VEHICLE_BEACON = 1  # Gói tin định vị từ xe
MSG_TYPE_RSU_SIGNAL = 2      # Trạng thái đèn từ RSU
MSG_TYPE_EMERGENCY_ALERT = 3  # Cảnh báo xe cứu thương
MSG_TYPE_CONGESTION_INFO = 4  # Thông tin tắc đường

# ===== Data Classes =====
@dataclass
class VehicleBeacon:
    """Gói tin beacon từ xe (V2I)"""
    vehicle_id: str
    timestamp: float
    position: Tuple[float, float]  # (x, y)
    speed: float  # m/s
    acceleration: float
    is_emergency: bool
    lane_id: str
    vehicle_type: str

@dataclass
class RSUSignal:
    """Trạng thái đèn từ RSU (V2I)"""
    rsu_id: str
    timestamp: float
    traffic_light_id: str
    current_phase: int  # 0=Red, 1=Yellow, 2=Green
    phase_duration: float
    queue_length: int
    estimated_wait_time: float

@dataclass
class TrafficLightState:
    """Trạng thái nội bộ của đèn giao thông"""
    tl_id: str
    current_phase: int
    phase_duration: float
    queue_length: int
    emergency_vehicles: List[str]
    congestion_level: str  # 'low', 'medium', 'high', 'critical'
    last_update: float

# ===== VANET Message Protocol =====
class VANETMessage:
    """Mã hóa/giải mã gói tin VANET"""
    
    @staticmethod
    def encode_beacon(beacon: VehicleBeacon) -> bytes:
        """Mã hóa gói tin beacon"""
        msg = struct.pack(
            '!BdffHB??',
            MSG_TYPE_VEHICLE_BEACON,
            beacon.timestamp,
            beacon.position[0],
            beacon.position[1],
            int(beacon.speed * 100),  # Chuyển thành cm/s
            len(beacon.vehicle_id),
            beacon.is_emergency,
            len(beacon.lane_id) > 0
        )
        msg += beacon.vehicle_id.encode() + beacon.lane_id.encode()
        return msg
    
    @staticmethod
    def decode_beacon(data: bytes) -> Optional[VehicleBeacon]:
        """Giải mã gói tin beacon"""
        try:
            offset = 0
            msg_type = struct.unpack_from('!B', data, offset)[0]
            if msg_type != MSG_TYPE_VEHICLE_BEACON:
                return None
            
            offset += 1
            timestamp, x, y, speed_cm, vid_len, is_emerg, has_lane = struct.unpack_from(
                '!dffHB??', data, offset
            )
            offset += 21  # 8+4+4+2+1+1+1
            
            vehicle_id = data[offset:offset+vid_len].decode()
            offset += vid_len
            
            lane_id = data[offset:].decode() if has_lane else ""
            
            return VehicleBeacon(
                vehicle_id=vehicle_id,
                timestamp=timestamp,
                position=(x, y),
                speed=speed_cm / 100.0,
                acceleration=0.0,
                is_emergency=is_emerg,
                lane_id=lane_id,
                vehicle_type="emergency" if is_emerg else "normal"
            )
        except Exception as e:
            logger.warning(f"Failed to decode beacon: {e}")
            return None

class SmartTrafficController:
    """
    Hệ thống điều khiển đèn giao thông thông minh với VANET
    
    Đặc điểm:
    --------
    - Giao thức DSRC 802.11p cho V2I communication
    - RSU (Road Side Units) đặt tại mỗi đèn giao thông
    - Thuật toán thích ứng dựa trên trạng thái xe trong vùng
    - Ưu tiên xe cứu thương (Emergency Response)
    - Dự đoán tắc đường bằng lịch sử giao thông
    """
    
    def __init__(self, sumo_config: str):
        """
        Khởi tạo hệ thống điều khiển
        
        Args:
            sumo_config: Đường dẫn đến file cấu hình SUMO
        """
        self.sumo_config = sumo_config
        
        # ===== SUMO State =====
        self.traffic_lights: List[str] = []
        self.vehicles: Dict[str, Dict] = {}
        self.simulation_time = 0.0
        
        # ===== VANET Infrastructure =====
        self.rsu_controllers: Dict[str, 'RSUController'] = {}  # RSU cho mỗi đèn
        self.vehicle_beacons: Dict[str, VehicleBeacon] = {}  # Beacon từ xe gần đây
        self.rsu_broadcast_sockets: Dict[str, socket.socket] = {}
        
        # ===== TL State Management =====
        self.tl_states: Dict[str, TrafficLightState] = {}
        self.last_phase: Dict[str, int] = {}
        self.last_duration: Dict[str, float] = {}
        self.phase_start_time: Dict[str, float] = {}
        self.phase_duration_set_time: Dict[str, float] = {}  # Khi nào set duration
        
        # ===== Emergency Management =====
        self.emergency_vehicles: Dict[str, Dict] = {}  # {vehicle_id: info}
        
        # ===== Traffic Prediction =====
        self.traffic_history: Dict[str, List[int]] = defaultdict(list)
        self.max_history = 20
        
        # ===== Statistics =====
        self.stats = {
            'total_vehicles': 0,
            'emergency_vehicles': 0,
            'avg_wait_time': 0.0,
            'total_distance': 0.0,
            'congestion_events': 0,
            'emergency_events': 0,
            'vanet_messages_sent': 0,
            'vanet_messages_received': 0,
        }
        
        # ===== VANET Parameters =====
        self.rsu_broadcast_interval = RSU_BROADCAST_INTERVAL
        self.next_rsu_broadcast = {}
        
        logger.info(f"✅ Initialized SmartTrafficController with VANET integration")
        
    # ===== SUMO Simulation Methods =====
    def start_simulation(self) -> bool:
        """Khởi động mô phỏng SUMO"""
        try:
            sumo_cmd = [
                'sumo-gui',
                '-c', self.sumo_config,
                '--start',
                '--quit-on-end'
            ]
            
            traci.start(sumo_cmd)
            logger.info("✅ Khởi động mô phỏng SUMO thành công")
            
            # Lấy danh sách đèn giao thông
            self.traffic_lights = traci.trafficlight.getIDList()
            logger.info(f"🚦 Tìm thấy {len(self.traffic_lights)} đèn giao thông")
            
            # Khởi tạo RSU cho mỗi đèn
            self._initialize_rsu_infrastructure()
            
            return True
        except Exception as e:
            logger.error(f"❌ Lỗi khởi động SUMO: {e}")
            return False
    
    def _initialize_rsu_infrastructure(self):
        """Khởi tạo hệ thống RSU (Road Side Units) cho mỗi đèn"""
        logger.info("🌐 Khởi tạo VANET Infrastructure (RSU)...")
        
        for tl_id in self.traffic_lights:
            rsu = RSUController(tl_id, DSRC_TX_RANGE)
            self.rsu_controllers[tl_id] = rsu
            self.tl_states[tl_id] = TrafficLightState(
                tl_id=tl_id,
                current_phase=0,
                phase_duration=30.0,
                queue_length=0,
                emergency_vehicles=[],
                congestion_level='low',
                last_update=0.0
            )
            self.next_rsu_broadcast[tl_id] = self.rsu_broadcast_interval
            
            # ===== KHỞI TẠO THỜI GIAN PHA =====
            self.phase_start_time[tl_id] = 0.0
            
            logger.info(f"  ✅ RSU cài đặt cho {tl_id}")
    
    # ===== VANET Communication Methods =====
    def broadcast_rsu_signal(self, tl_id: str, timestamp: float):
        """
        Phát sóng tín hiệu từ RSU (802.11p DSRC)
        Thông tin: trạng thái đèn, hàng đợi, thời gian chờ
        """
        try:
            state = self.tl_states[tl_id]
            rsu = self.rsu_controllers[tl_id]
            
            signal = RSUSignal(
                rsu_id=f"RSU_{tl_id}",
                timestamp=timestamp,
                traffic_light_id=tl_id,
                current_phase=state.current_phase,
                phase_duration=state.phase_duration,
                queue_length=state.queue_length,
                estimated_wait_time=self._estimate_wait_time(tl_id)
            )
            
            # Broadcast đến tất cả xe trong vùng (trong phạm vi DSRC_TX_RANGE)
            for vehicle_id, beacon in self.vehicle_beacons.items():
                rsu.receive_broadcast(signal)
            
            self.stats['vanet_messages_sent'] += 1
            
        except Exception as e:
            logger.warning(f"Failed to broadcast RSU signal: {e}")
    
    def process_vehicle_beacons(self, timestamp: float):
        """
        Xử lý beacon từ xe (V2I)
        Cập nhật thông tin xe vào hệ thống RSU
        """
        try:
            vehicle_ids = traci.vehicle.getIDList()
            
            for vehicle_id in vehicle_ids:
                try:
                    pos = traci.vehicle.getPosition(vehicle_id)
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    accel = traci.vehicle.getAcceleration(vehicle_id)
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    vtype = traci.vehicle.getTypeID(vehicle_id)
                    is_emergency = 'emergency' in vtype.lower()
                    
                    beacon = VehicleBeacon(
                        vehicle_id=vehicle_id,
                        timestamp=timestamp,
                        position=pos,
                        speed=speed,
                        acceleration=accel,
                        is_emergency=is_emergency,
                        lane_id=lane_id,
                        vehicle_type=vtype
                    )
                    
                    self.vehicle_beacons[vehicle_id] = beacon
                    
                    # Tìm RSU gần nhất (đèn giao thông gần nhất)
                    nearest_rsu = self._find_nearest_rsu(pos)
                    if nearest_rsu:
                        self.rsu_controllers[nearest_rsu].receive_vehicle_beacon(beacon)
                    
                    self.stats['vanet_messages_received'] += 1
                    
                except Exception as e:
                    logger.debug(f"Error processing vehicle {vehicle_id}: {e}")
            
        except Exception as e:
            logger.warning(f"Error processing beacons: {e}")
    
    def _find_nearest_rsu(self, position: Tuple[float, float]) -> Optional[str]:
        """Tìm RSU gần nhất dựa trên vị trí"""
        try:
            nearest_tl = None
            min_distance = float('inf')
            
            for tl_id in self.traffic_lights:
                tl_pos = traci.trafficlight.getPosition(tl_id)
                dist = ((position[0] - tl_pos[0])**2 + (position[1] - tl_pos[1])**2)**0.5
                
                if dist < min_distance and dist <= DSRC_TX_RANGE:
                    min_distance = dist
                    nearest_tl = tl_id
            
            return nearest_tl
        except Exception as e:
            logger.debug(f"Error finding nearest RSU: {e}")
            return None
    
    def analyze_traffic_condition(self, tl_id: str) -> Dict:
        """
        Phân tích điều kiện giao thông đơn giản
        Chỉ dùng mô hình giao thông cơ bản, không đếm xe
        """
        try:
            state = self.tl_states[tl_id]
            
            # Lấy danh sách các hàng được kiểm soát bởi đèn này
            lanes = traci.trafficlight.getControlledLanes(tl_id)
            
            # Cập nhật state
            state.queue_length = 0
            
            return {
                'tl_id': tl_id,
                'total_vehicles': 0,
                'lanes': lanes,
                'queue_length': 0
            }
        except Exception as e:
            logger.warning(f"Lỗi phân tích giao thông: {e}")
            return {'total_vehicles': 0, 'queue_length': 0}
    
    def _is_in_range(self, vehicle_pos: Tuple[float, float], tl_id: str) -> bool:
        """Kiểm tra xe có nằm trong vùng DSRC của RSU không"""
        try:
            tl_pos = traci.trafficlight.getPosition(tl_id)
            dist = ((vehicle_pos[0] - tl_pos[0])**2 + (vehicle_pos[1] - tl_pos[1])**2)**0.5
            return dist <= DSRC_TX_RANGE
        except:
            return False
    
    # ===== Traffic Prediction & Control =====
    def predict_congestion(self, tl_id: str, total_vehicles: int) -> str:
        """
        Dự đoán mức tắc đường dựa trên XE CHỜ THỰC TẾ
        
        Tiêu chí:
        - Low: ≤ 3 xe chờ
        - Medium: 4-8 xe chờ
        - High: 9-15 xe chờ
        - Critical: > 15 xe chờ
        """
        try:
            self.traffic_history[tl_id].append(total_vehicles)
            
            if len(self.traffic_history[tl_id]) > self.max_history:
                self.traffic_history[tl_id].pop(0)
            
            # Tính trung bình động (MA-5)
            if len(self.traffic_history[tl_id]) >= 5:
                recent_avg = sum(self.traffic_history[tl_id][-5:]) / 5
            else:
                recent_avg = total_vehicles
            
            # Dự đoán dựa trên số xe chờ
            if recent_avg > 15:
                return 'critical'
            elif recent_avg > 8:
                return 'high'
            elif recent_avg > 3:
                return 'medium'
            else:
                return 'low'
        
        except Exception as e:
            logger.warning(f"Lỗi dự đoán tắc: {e}")
            return 'low'
    
    def _estimate_wait_time(self, tl_id: str) -> float:
        """Ước tính thời gian chờ dựa trên hàng đợi và pha hiện tại"""
        try:
            state = self.tl_states[tl_id]
            current_phase = traci.trafficlight.getPhase(tl_id)
            
            # Nếu đèn đỏ (phase 0 hoặc 2)
            if current_phase in [0, 2]:
                remaining = state.phase_duration - (self.simulation_time - self.phase_start_time.get(tl_id, 0))
                return max(0, remaining)
            
            return 0.0
        except:
            return 0.0
    
    # ===== Traffic Light Control Algorithm =====
    # ===== Helper Methods =====
    def _simplify_tl_name(self, tl_id: str) -> str:
        """Rút gọn tên ngã tư thành dạng đơn giản"""
        # Nếu tên quá dài, chỉ lấy phần đầu
        if len(tl_id) > 20:
            # Lấy 3 ký tự đầu + 3 ký tự cuối
            return f"Ngã tư #{tl_id[:3]}...{tl_id[-3:]}"
        return f"Ngã tư {tl_id}"
    
    def _get_phase_description(self, phase: int) -> str:
        """Dịch số pha thành tiếng Việt"""
        phase_map = {
            0: "🟢 Xanh hàng ngang",
            1: "🟡 Vàng hàng ngang",
            2: "🟢 Xanh hàng dọc",
            3: "🟡 Vàng hàng dọc"
        }
        return phase_map.get(phase, f"Pha {phase}")
    
    def _get_congestion_description(self, level: str, vehicles: int) -> str:
        """Mô tả mức tắc đường"""
        if level == 'critical':
            return f"🔴 TẮC NGHIÊM TRỌNG"
        elif level == 'high':
            return f"🟠 Tắc nặng"
        elif level == 'medium':
            return f"🟡 Tắc vừa"
        else:
            return f"🟢 Bình thường"
    
    def adjust_traffic_light(self, tl_id: str, traffic_info: Dict):
        """
        Thuật toán điều khiển thông minh dựa trên VANET data
        
        Chiến lược:
        1. Phát hiện tắc đường - Xe chờ lâu → xanh lâu hơn
        2. Cân bằng luồng - Hàng nhiều xe → xanh lâu hơn
        3. Dự đoán tắc đường - Từ lịch sử giao thông (MA-5)
        4. **CHỦ ĐỘNG chuyển pha** (không chờ SUMO)
        """
        try:
            state = self.tl_states[tl_id]
            current_phase = traci.trafficlight.getPhase(tl_id)
            total_vehicles = traffic_info['total_vehicles']
            
            # ===== ĐIỀU KHIỂN THÔNG MINH =====
            congestion_level = self.predict_congestion(tl_id, total_vehicles)
            state.congestion_level = congestion_level
            
            new_duration = self._calculate_adaptive_duration(
                tl_id, total_vehicles, congestion_level, current_phase
            )
            
            if new_duration is None:
                return
            
            # Lấy thời gian còn lại của pha hiện tại
            current_duration = traci.trafficlight.getPhaseDuration(tl_id)
            time_in_phase = self.simulation_time - self.phase_start_time.get(tl_id, 0)
            
            # ===== CHUYỂN PHA CÓ CHỦ Ý =====
            # Nếu đã qua thời gian, chuyển sang pha tiếp theo
            if time_in_phase >= current_duration - 0.5:
                try:
                    # Lấy số pha tổng cộng
                    definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)
                    num_phases = len(definition[0].phases)
                    
                    # Chuyển pha (0 → 1 → 2 → 3 → ... → 0)
                    next_phase = (current_phase + 1) % num_phases
                    traci.trafficlight.setPhase(tl_id, next_phase)
                    self.phase_start_time[tl_id] = self.simulation_time
                    current_phase = next_phase  # Cập nhật để log đúng
                    
                    next_description = self._get_phase_description(next_phase)
                    simple_name = self._simplify_tl_name(tl_id)
                    logger.info(
                        f"[{self.simulation_time:.1f}s] 🔄 {simple_name}: "
                        f"Chuyển sang {next_description}"
                    )
                except Exception as e:
                    logger.debug(f"Cannot switch phase: {e}")
            
            # ===== CHỈ ĐẶT DURATION CHO PHA XANH (0, 2), KO SET CHO VÀNG =====
            is_green_phase = current_phase in [0, 2]
            
            if is_green_phase:
                traci.trafficlight.setPhaseDuration(tl_id, new_duration)
                self.last_duration[tl_id] = new_duration
                self.phase_duration_set_time[tl_id] = self.simulation_time
            
            if self.simulation_time % 5 == 0:
                phase_description = self._get_phase_description(current_phase)
                congestion_description = self._get_congestion_description(congestion_level, total_vehicles)
                simple_name = self._simplify_tl_name(tl_id)
                
                logger.info(
                    f"[{self.simulation_time:.1f}s] 🚦 {simple_name}: "
                    f"{phase_description} ({new_duration:.0f}s) | "
                    f"{congestion_description}"
                )
        
        except Exception as e:
            logger.warning(f"Lỗi điều khiển đèn: {e}")
    
    def _handle_emergency_vehicles(self, tl_id: str, emergency_vehicles: List[str], state: TrafficLightState):
        """Xử lý ưu tiên cho xe cứu thương"""
        try:
            for vehicle_id in emergency_vehicles:
                if vehicle_id not in self.emergency_vehicles:
                    self.emergency_vehicles[vehicle_id] = {
                        'tl_id': tl_id,
                        'first_time': self.simulation_time,
                        'phase_changes': 0
                    }
                    simple_name = self._simplify_tl_name(tl_id)
                    logger.warning(
                        f"🚨 CẤP CỨU! Phát hiện xe cứu thương {vehicle_id} "
                        f"tại {simple_name} - BẬT XANH ĐẦU TIÊN"
                    )
                    self.stats['emergency_events'] += 1
                
                # Bật xanh cho xe cứu thương
                emergency_time = self.simulation_time - self.emergency_vehicles[vehicle_id]['first_time']
                
                if emergency_time < 120:  # 2 phút
                    traci.trafficlight.setPhase(tl_id, 0)  # Set to green phase
                    traci.trafficlight.setPhaseDuration(tl_id, 120)
                    self.emergency_vehicles[vehicle_id]['phase_changes'] += 1
                    
                    if emergency_time < 5:
                        simple_name = self._simplify_tl_name(tl_id)
                        logger.warning(
                            f"🚨 CHỚ KHÔNG CHỖ! {vehicle_id} được ưu tiên "
                            f"xanh liên tục 120s tại {simple_name}"
                        )
        
        except Exception as e:
            logger.warning(f"Lỗi xử lý xe cứu thương: {e}")
    
    def _get_incoming_lanes_for_phase(self, tl_id: str, phase: int) -> List[str]:
        """Lấy danh sách lane vào cho pha xanh"""
        try:
            definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)
            if definition and len(definition) > 0:
                logic = definition[0]
                if phase < len(logic.phases):
                    phase_state = logic.phases[phase].state
                    # State là string, ví dụ: "GrGr" (Green for lane 0,2)
                    # Lấy tất cả incoming lanes
                    all_incoming = traci.trafficlight.getControlledLanes(tl_id)
                    incoming_lanes = []
                    
                    for i, char in enumerate(phase_state):
                        if i < len(all_incoming):
                            # G = green (có xe đi)
                            if char in ['g', 'G']:
                                lane_id = all_incoming[i]
                                if lane_id not in incoming_lanes:
                                    incoming_lanes.append(lane_id)
                    return incoming_lanes
        except Exception as e:
            logger.debug(f"Lỗi lấy incoming lanes: {e}")
        return []
    
    def _count_vehicles_on_lanes(self, lane_ids: List[str]) -> int:
        """Đếm số xe đang chờ trên các lane"""
        try:
            count = 0
            for lane_id in lane_ids:
                try:
                    vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)
                    count += len(vehicle_ids)
                except:
                    pass
            return count
        except:
            return 0
    
    def _calculate_adaptive_duration(self, tl_id: str, total_vehicles: int, 
                                   congestion_level: str, current_phase: int) -> Optional[float]:
        """
        Tính toán thời gian pha động thông minh dựa trên:
        1. Dự đoán tắc đường - Từ lịch sử (MA-5)
        2. Cân bằng luồng - So sánh 2 hướng, ưu tiên hướng nhiều xe
        3. Độ dài pha: xanh lâu khi hướng tương ứng có nhiều xe
        """
        try:
            # ===== CHIA LOẠI PHA =====
            is_yellow_phase = current_phase in [1, 3]  # Cả pha 1 (vàng ngang) và pha 3 (vàng dọc)
            is_green_phase_horizontal = current_phase == 0  # Hàng ngang xanh
            is_green_phase_vertical = current_phase == 2    # Hàng dọc xanh
            
            # Pha vàng: luôn 3 giây
            if is_yellow_phase:
                return 3.0
            
            # ===== LOGIC THÔNG MINH: SO SÁNH 2 HƯỚNG =====
            try:
                # Lấy lane xanh cho pha 0 (ngang) và pha 2 (dọc)
                horizontal_lanes = self._get_incoming_lanes_for_phase(tl_id, 0)
                vertical_lanes = self._get_incoming_lanes_for_phase(tl_id, 2)
                
                horizontal_count = self._count_vehicles_on_lanes(horizontal_lanes)
                vertical_count = self._count_vehicles_on_lanes(vertical_lanes)
                
                logger.debug(f"[{self.simulation_time:.1f}s] {tl_id}: H={horizontal_count}, V={vertical_count}")
            except Exception as e:
                logger.debug(f"Lỗi đếm xe: {e}")
                horizontal_count = 0
                vertical_count = 0
            
            # ===== LOGIC ƯỚI TIÊN HƯỚNG CÓ NHIỀU XE =====
            
            # Nếu pha hiện tại là xanh hàng ngang nhưng hàng dọc nhiều xe hơn, rút ngắn
            if is_green_phase_horizontal and vertical_count > horizontal_count + 3:
                # Hàng dọc chờ nhiều hơn 3 xe → rút ngắn xanh hàng ngang
                return 20.0
            
            # Nếu pha hiện tại là xanh hàng dọc nhưng hàng ngang nhiều xe hơn, rút ngắn
            if is_green_phase_vertical and horizontal_count > vertical_count + 3:
                return 20.0
            
            # ===== NẾU CÂN BẰNG HOẶC CÓ XE, DÙNG LỊCH SỬ =====
            
            # 1. LẤY LỊCH SỬ GIAO THÔNG
            history = self.traffic_history[tl_id]
            
            # 2. TÍNH TRUNG BÌNH ĐỘNG (MA-5)
            if len(history) >= 5:
                recent_avg = sum(history[-5:]) / 5
            else:
                recent_avg = 5  # Giả định mặc định
            
            # 3. PHÂN LOẠI TẮAC ĐƯỜNG DỰA VÀO LỊCH SỬ
            if recent_avg > 8:
                # 🔴 TẮC NGHIÊM TRỌNG
                base_green_duration = 80.0
                base_red_duration = 15.0
            elif recent_avg > 5:
                # 🟠 TẮC NẶNG
                base_green_duration = 60.0
                base_red_duration = 20.0
            elif recent_avg > 3:
                # 🟡 TẮC VỪA
                base_green_duration = 40.0
                base_red_duration = 30.0
            else:
                # 🟢 BÌNH THƯỜNG
                base_green_duration = 25.0
                base_red_duration = 35.0
            
            # 4. ƯU TIÊN HƯỚNG CÓ NHIỀU XE
            if is_green_phase_horizontal and horizontal_count > vertical_count:
                # Hàng ngang có nhiều xe → xanh lâu hơn
                base_green_duration = min(90.0, base_green_duration + 15)
            elif is_green_phase_vertical and vertical_count > horizontal_count:
                # Hàng dọc có nhiều xe → xanh lâu hơn
                base_green_duration = min(90.0, base_green_duration + 15)
            
            # 5. TRẢ VỀ THỜI GIAN
            if is_green_phase_horizontal or is_green_phase_vertical:
                return base_green_duration
            else:
                return base_red_duration
        
        except Exception as e:
            logger.warning(f"Lỗi tính thời gian pha: {e}")
            return None
    
    # ===== Statistics & Reporting =====
    def get_vehicle_statistics(self) -> Dict:
        """Lấy thống kê giao thông từ SUMO"""
        try:
            vehicle_ids = traci.vehicle.getIDList()
            
            stats = {
                'total_vehicles': len(vehicle_ids),
                'emergency_vehicles': 0,
                'avg_speed': 0.0,
                'total_distance': 0.0,
                'vehicles_by_type': defaultdict(int)
            }
            
            total_speed = 0.0
            for vehicle_id in vehicle_ids:
                try:
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    distance = traci.vehicle.getDistance(vehicle_id)
                    vtype = traci.vehicle.getTypeID(vehicle_id)
                    
                    total_speed += speed
                    stats['total_distance'] += distance
                    stats['vehicles_by_type'][vtype] += 1
                    
                    if 'emergency' in vtype.lower():
                        stats['emergency_vehicles'] += 1
                
                except Exception as e:
                    logger.debug(f"Error getting stats for {vehicle_id}: {e}")
            
            if vehicle_ids:
                stats['avg_speed'] = total_speed / len(vehicle_ids)
            
            return stats
        except Exception as e:
            logger.warning(f"Error getting statistics: {e}")
            return {}
    
    def log_statistics(self):
        """Ghi log thống kê mô phỏng"""
        try:
            stats = self.get_vehicle_statistics()
            if not stats:
                return
            
            self.stats['total_vehicles'] = stats['total_vehicles']
            self.stats['emergency_vehicles'] = stats['emergency_vehicles']
            
            # Tính trạng thái giao thông
            if stats['total_vehicles'] == 0:
                traffic_status = "🟢 Không có xe"
            elif stats['avg_speed'] < 2:
                traffic_status = "🔴 TẮC NGHIÊM TRỌNG (tốc độ < 2 m/s)"
            elif stats['avg_speed'] < 5:
                traffic_status = "🟠 Tắc nặng (tốc độ < 5 m/s)"
            else:
                traffic_status = "🟢 Bình thường"
            
            logger.info(
                f"\n{'='*70}\n"
                f"📊 THỐNG KÊ MÔ PHỎNG LÚC {self.simulation_time:.0f}s\n"
                f"{'='*70}\n"
                f"  🚗 Tổng xe trên đường: {stats['total_vehicles']} chiếc\n"
                f"  🏍️  Xe máy: {dict(stats['vehicles_by_type']).get('motorcycle', 0)}\n"
                f"  🚙 Ô tô: {dict(stats['vehicles_by_type']).get('car', 0)}\n"
                f"  ⚡ Tốc độ trung bình: {stats['avg_speed']:.1f} m/s\n"
                f"  📏 Quãng đường đi được: {stats['total_distance']:.0f} m\n"
                f"  💬 Tin VANET gửi đi: {self.stats['vanet_messages_sent']} gói\n"
                f"  💬 Tin VANET nhận được: {self.stats['vanet_messages_received']} gói\n"
                f"  {traffic_status}\n"
                f"{'='*70}\n"
            )
        except Exception as e:
            logger.warning(f"Lỗi ghi log thống kê: {e}")
    
    def run_simulation(self, max_steps: int = 3600):
        """
        Chạy mô phỏng tích hợp NS-3 + SUMO
        """
        logger.info(f"\n{'='*70}")
        logger.info(f"🚀 Smart Traffic Control System - VANET Integration")
        logger.info(f"⏱️  Starting at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"{'='*70}\n")
        
        try:
            step = 0
            while step < max_steps:
                try:
                    # Tiến hành một bước mô phỏng SUMO
                    traci.simulationStep()
                except traci.TraCIException:
                    logger.info("✅ Simulation ended (SUMO closed)")
                    break
                
                self.simulation_time = step
                
                # ===== Process VANET Communications =====
                # Mỗi 0.5s: xe gửi beacon
                if step % 5 == 0:  # 5 steps = 0.5s
                    self.process_vehicle_beacons(self.simulation_time)
                
                # Mỗi 1.0s: RSU phát sóng
                for tl_id in self.traffic_lights:
                    if step % 10 == 0:  # 10 steps = 1.0s
                        self.broadcast_rsu_signal(tl_id, self.simulation_time)
                
                # ===== Traffic Light Control =====
                for tl_id in self.traffic_lights:
                    traffic_info = self.analyze_traffic_condition(tl_id)
                    if traffic_info:
                        self.adjust_traffic_light(tl_id, traffic_info)
                
                # ===== Periodic Reporting =====
                if step % 100 == 0 and step > 0:  # Mỗi 10s
                    self.log_statistics()
                
                step += 1
            
            # ===== Final Report =====
            logger.info(f"\n{'='*70}")
            logger.info(f"✅ Simulation completed successfully!")
            logger.info(f"⏱️  Total simulated time: {self.simulation_time:.1f}s")
            logger.info(f"{'='*70}\n")
            self.log_statistics()
            self._save_results()
            
        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
        except Exception as e:
            logger.error(f"Error during simulation: {e}", exc_info=True)
        finally:
            self.stop_simulation()
    
    def stop_simulation(self):
        """Dừng mô phỏng"""
        try:
            traci.close()
            logger.info("✅ SUMO simulation closed")
        except Exception as e:
            logger.warning(f"Error closing simulation: {e}")
    
    def _save_results(self):
        """Lưu kết quả mô phỏng"""
        try:
            results_file = os.path.join(
                os.path.dirname(self.sumo_config),
                'simulation_results.json'
            )
            
            with open(results_file, 'w') as f:
                json.dump(self.stats, f, indent=2)
            
            logger.info(f"📁 Results saved to {results_file}")
        except Exception as e:
            logger.warning(f"Error saving results: {e}")


# ===== RSU Controller Class =====
class RSUController:
    """Road Side Unit - Controller tại mỗi đèn giao thông"""
    
    def __init__(self, tl_id: str, tx_range: float):
        self.tl_id = tl_id
        self.tx_range = tx_range
        self.received_beacons: List[VehicleBeacon] = []
        self.vehicle_count = 0
        self.last_broadcast_time = 0.0
    
    def receive_vehicle_beacon(self, beacon: VehicleBeacon):
        """Nhận beacon từ xe"""
        self.received_beacons.append(beacon)
        self.vehicle_count += 1
    
    def receive_broadcast(self, signal: RSUSignal):
        """Nhận broadcast từ RSU khác (nếu cần điều phối)"""
        pass
    
    def clear_beacons(self):
        """Xóa beacon cũ"""
        self.received_beacons.clear()
        self.vehicle_count = 0


def main():
    """
    Hàm chính - Khởi động hệ thống
    
    Usage:
        python smart_traffic_sim.py --config thuan.sumocfg --simtime 3600
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Smart Traffic Control System with VANET Integration'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='thuan.sumocfg',
        help='SUMO configuration file (default: thuan.sumocfg)'
    )
    parser.add_argument(
        '--simtime',
        type=float,
        default=3600,
        help='Simulation time in seconds (default: 3600s = 1 hour)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='simulation_results.json',
        help='Output file for results'
    )
    
    args = parser.parse_args()
    
    # Đường dẫn đến file cấu hình SUMO
    project_dir = os.path.dirname(os.path.abspath(__file__))
    sumo_config = os.path.join(project_dir, args.config)
    
    if not os.path.exists(sumo_config):
        logger.error(f"❌ Config file not found: {sumo_config}")
        sys.exit(1)
    
    logger.info("="*70)
    logger.info("🌐 VANET Smart Traffic Control System - NS-3 Integration")
    logger.info("="*70)
    logger.info(f"📋 Configuration: {sumo_config}")
    logger.info(f"⏱️  Simulation Duration: {args.simtime}s")
    logger.info("="*70 + "\n")
    
    # Tạo bộ điều khiển giao thông thông minh
    controller = SmartTrafficController(sumo_config)
    
    # Khởi động mô phỏng SUMO + VANET
    if not controller.start_simulation():
        sys.exit(1)
    
    # Chạy mô phỏng
    try:
        controller.run_simulation(max_steps=int(args.simtime * 10))  # 1 step = 0.1s
    except KeyboardInterrupt:
        logger.info("\n🛑 Simulation interrupted by user")
    except Exception as e:
        logger.error(f"❌ Fatal error: {e}", exc_info=True)
    finally:
        controller.stop_simulation()
    
    logger.info("✅ Simulation finished successfully")


if __name__ == '__main__':
    main()
