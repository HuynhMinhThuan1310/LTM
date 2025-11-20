#!/usr/bin/env python3
"""
======================================================================
Smart Traffic Light Control System with VANET + NS-3 Integration
Hệ thống điều khiển đèn giao thông thông minh sử dụng VANET 802.11p
======================================================================

FIXED VERSION - Compatible with thuan.sumocfg
"""

import os
import sys
import time
import struct
from datetime import datetime
from collections import defaultdict
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
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
DSRC_TX_RANGE = 300  # meters
VEHICLE_BEACON_INTERVAL = 0.5  # seconds
RSU_BROADCAST_INTERVAL = 1.0  # seconds

# ===== Message Types =====
MSG_TYPE_VEHICLE_BEACON = 1
MSG_TYPE_RSU_SIGNAL = 2

# ===== Data Classes =====
@dataclass
class VehicleBeacon:
    """Gói tin beacon từ xe (V2I)"""
    vehicle_id: str
    timestamp: float
    position: Tuple[float, float]
    speed: float
    is_emergency: bool
    lane_id: str

@dataclass
class TrafficLightState:
    """Trạng thái nội bộ của đèn giao thông"""
    tl_id: str
    current_phase: int
    phase_duration: float
    queue_length: int
    last_update: float


class SmartTrafficController:
    """
    Hệ thống điều khiển đèn giao thông thông minh với VANET
    """
    
    def __init__(self, sumo_config: str):
        self.sumo_config = sumo_config
        
        # ===== SUMO State =====
        self.traffic_lights: List[str] = []
        self.simulation_time = 0.0
        
        # ===== VANET Infrastructure =====
        self.vehicle_beacons: Dict[str, VehicleBeacon] = {}
        
        # ===== TL State Management =====
        self.tl_states: Dict[str, TrafficLightState] = {}
        self.phase_start_time: Dict[str, float] = {}
        self.last_duration: Dict[str, float] = {}
        self.last_log_phase_segment: Dict[str, int] = {}  # Track segment cuối log
        
        # ===== Traffic History (cho MA-5) =====
        self.traffic_history: Dict[str, List[int]] = defaultdict(list)
        self.max_history = 20
        
        # ===== Statistics =====
        self.stats = {
            'total_vehicles': 0,
            'emergency_vehicles': 0,
            'avg_speed': 0.0,
            'vanet_messages_sent': 0,
            'vanet_messages_received': 0,
        }
        
        logger.info(f"✅ Initialized SmartTrafficController")
        
    # ===== SUMO Simulation Methods =====
    def start_simulation(self) -> bool:
        """Khởi động mô phỏng SUMO"""
        try:
            sumo_cmd = [
                'sumo-gui',
                '-c', self.sumo_config,
                '--start',
                '--quit-on-end',
                '--step-length', '0.1'
            ]
            
            traci.start(sumo_cmd)
            logger.info("✅ Khởi động SUMO thành công")
            
            # Lấy danh sách đèn giao thông
            self.traffic_lights = traci.trafficlight.getIDList()
            logger.info(f"🚦 Tìm thấy {len(self.traffic_lights)} đèn giao thông")
            
            # Khởi tạo state
            self._initialize_traffic_lights()
            
            return True
        except Exception as e:
            logger.error(f"❌ Lỗi khởi động SUMO: {e}")
            return False
    
    def _initialize_traffic_lights(self):
        """Khởi tạo state cho các đèn"""
        for tl_id in self.traffic_lights:
            self.tl_states[tl_id] = TrafficLightState(
                tl_id=tl_id,
                current_phase=0,
                phase_duration=30.0,
                queue_length=0,
                last_update=0.0
            )
            self.phase_start_time[tl_id] = 0.0
            self.last_duration[tl_id] = 30.0
            
            # Tên đơn giản cho log
            simple_name = self._get_simple_name(tl_id)
            logger.info(f"  ✅ Khởi tạo đèn: {simple_name}")
    
    def _get_simple_name(self, tl_id: str) -> str:
        """Rút gọn tên đèn giao thông"""
        if "cluster" in tl_id:
            return "Ngã tư chính"
        return tl_id[:15] if len(tl_id) > 15 else tl_id
    
    # ===== VANET Communication =====
    def process_vehicle_beacons(self, timestamp: float):
        """Thu thập beacon từ các xe"""
        try:
            vehicle_ids = traci.vehicle.getIDList()
            
            for vehicle_id in vehicle_ids:
                try:
                    pos = traci.vehicle.getPosition(vehicle_id)
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    vtype = traci.vehicle.getTypeID(vehicle_id)
                    is_emergency = 'emergency' in vtype.lower()
                    
                    beacon = VehicleBeacon(
                        vehicle_id=vehicle_id,
                        timestamp=timestamp,
                        position=pos,
                        speed=speed,
                        is_emergency=is_emergency,
                        lane_id=lane_id
                    )
                    
                    self.vehicle_beacons[vehicle_id] = beacon
                    self.stats['vanet_messages_received'] += 1
                    
                except Exception as e:
                    logger.debug(f"Lỗi xử lý xe {vehicle_id}: {e}")
            
        except Exception as e:
            logger.warning(f"Lỗi thu thập beacon: {e}")
    
    # ===== Traffic Analysis =====
    def analyze_traffic_at_junction(self, tl_id: str) -> Dict:
        """
        Phân tích giao thông tại ngã tư
        Đếm xe trên các hướng vào (incoming lanes)
        """
        try:
            # Lấy tất cả lanes được kiểm soát
            controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
            
            # Đếm xe theo hướng
            east_west_count = 0  # Lanes từ East/West (610420095#0, -408162778#3)
            north_south_count = 0  # Lanes từ North/South (694768221#0, 694768222#0)
            
            total_vehicles = 0
            
            for lane in controlled_lanes:
                try:
                    # Đếm xe đang chờ (speed < 2 m/s)
                    vehicles_on_lane = traci.lane.getLastStepVehicleIDs(lane)
                    waiting = 0
                    
                    for veh in vehicles_on_lane:
                        speed = traci.vehicle.getSpeed(veh)
                        if speed < 2.0:  # Xe đang chờ
                            waiting += 1
                    
                    total_vehicles += waiting
                    
                    # Phân loại hướng dựa vào tên lane
                    if '610420095' in lane or '408162778' in lane:
                        east_west_count += waiting
                    elif '694768221' in lane or '694768222' in lane:
                        north_south_count += waiting
                    
                except Exception as e:
                    logger.debug(f"Lỗi đếm xe trên lane {lane}: {e}")
            
            return {
                'tl_id': tl_id,
                'total_vehicles': total_vehicles,
                'east_west_count': east_west_count,
                'north_south_count': north_south_count,
                'lanes': controlled_lanes
            }
            
        except Exception as e:
            logger.warning(f"Lỗi phân tích giao thông: {e}")
            return {
                'total_vehicles': 0,
                'east_west_count': 0,
                'north_south_count': 0
            }
    
    # ===== Traffic Light Control =====
    def adjust_traffic_light(self, tl_id: str, traffic_info: Dict):
        """
        Điều khiển đèn thông minh
        
        Logic:
        1. Pha vàng (1, 3) → luôn 3 giây
        2. Pha xanh (0, 2) → thời gian động dựa trên:
           - Số xe chờ 2 hướng (East-West vs North-South)
           - Lịch sử giao thông (MA-5)
        """
        try:
            state = self.tl_states[tl_id]
            current_phase = traci.trafficlight.getPhase(tl_id)
            
            total_vehicles = traffic_info['total_vehicles']
            ew_count = traffic_info['east_west_count']
            ns_count = traffic_info['north_south_count']
            
            # ===== Lưu lịch sử =====
            self.traffic_history[tl_id].append(total_vehicles)
            if len(self.traffic_history[tl_id]) > self.max_history:
                self.traffic_history[tl_id].pop(0)
            
            # ===== Chuyển pha tự động =====
            time_in_phase = self.simulation_time - self.phase_start_time.get(tl_id, 0)
            current_duration = traci.trafficlight.getPhaseDuration(tl_id)
            
            if time_in_phase >= current_duration - 0.5:
                # Lấy số pha
                definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)
                num_phases = len(definition[0].phases)
                
                # Chuyển pha
                next_phase = (current_phase + 1) % num_phases
                traci.trafficlight.setPhase(tl_id, next_phase)
                self.phase_start_time[tl_id] = self.simulation_time
                
                phase_desc = self._get_phase_name(next_phase)
                simple_name = self._get_simple_name(tl_id)
                logger.info(
                    f"[{self.simulation_time:.1f}s] 🔄 {simple_name}: "
                    f"Chuyển sang {phase_desc}"
                )
                
                current_phase = next_phase
            
            # ===== Tính thời gian pha mới =====
            new_duration = self._calculate_phase_duration(
                current_phase, total_vehicles, ew_count, ns_count
            )
            
            # Chỉ set duration cho pha xanh
            if current_phase in [0, 2] and new_duration is not None:
                traci.trafficlight.setPhaseDuration(tl_id, new_duration)
                self.last_duration[tl_id] = new_duration
            
            # ===== Log 3 lần/chu kỳ pha (chia thành 3 đoạn) =====
            phase_duration = traci.trafficlight.getPhaseDuration(tl_id)
            if phase_duration > 0:
                log_interval = phase_duration / 3.0  # Chia thành 3 phần
                current_segment = int(time_in_phase / log_interval)  # 0, 1 hoặc 2
                
                # Log chỉ khi vào segment mới
                last_segment = self.last_log_phase_segment.get(tl_id, -1)
                if current_segment != last_segment and current_segment < 3:
                    self.last_log_phase_segment[tl_id] = current_segment
                    self._log_traffic_status(tl_id, current_phase, new_duration, 
                                            ew_count, ns_count, total_vehicles)
        
        except Exception as e:
            logger.warning(f"Lỗi điều khiển đèn: {e}")
    
    def _calculate_phase_duration(self, phase: int, total_vehicles: int,
                                  ew_count: int, ns_count: int) -> Optional[float]:
        """
        Tính thời gian pha động
        
        Phase 0: Xanh East-West
        Phase 1: Vàng East-West (3s)
        Phase 2: Xanh North-South
        Phase 3: Vàng North-South (3s)
        """
        # Pha vàng luôn 3 giây
        if phase in [1, 3]:
            return 3.0
        
        # Tính MA-5 (Moving Average 5 samples)
        history = self.traffic_history.get('cluster_420249144_6518482571_6518482574_6524986719_#2more', [])
        if len(history) >= 5:
            recent_avg = sum(history[-5:]) / 5.0
        else:
            recent_avg = total_vehicles if total_vehicles > 0 else 5
        
        # Base duration dựa trên lịch sử
        if recent_avg > 12:
            base_duration = 60.0  # Tắc nặng
        elif recent_avg > 8:
            base_duration = 50.0  # Tắc vừa
        elif recent_avg > 4:
            base_duration = 40.0  # Nhẹ
        else:
            base_duration = 30.0  # Thông thoáng
        
        # Điều chỉnh theo hướng hiện tại
        if phase == 0:  # Xanh East-West
            if ew_count > ns_count + 3:
                # East-West đông hơn → thêm thời gian
                base_duration = min(70.0, base_duration + 10)
            elif ns_count > ew_count + 3:
                # North-South đông hơn → rút ngắn
                base_duration = max(20.0, base_duration - 10)
        
        elif phase == 2:  # Xanh North-South
            if ns_count > ew_count + 3:
                # North-South đông hơn → thêm thời gian
                base_duration = min(70.0, base_duration + 10)
            elif ew_count > ns_count + 3:
                # East-West đông hơn → rút ngắn
                base_duration = max(20.0, base_duration - 10)
        
        return base_duration
    
    def _get_phase_name(self, phase: int) -> str:
        """Tên pha bằng tiếng Việt"""
        names = {
            0: "🟢 Xanh Đông-Tây",
            1: "🟡 Vàng Đông-Tây",
            2: "🟢 Xanh Bắc-Nam",
            3: "🟡 Vàng Bắc-Nam"
        }
        return names.get(phase, f"Pha {phase}")
    
    def _log_traffic_status(self, tl_id: str, phase: int, duration: float,
                           ew_count: int, ns_count: int, total: int):
        """Log trạng thái giao thông"""
        simple_name = self._get_simple_name(tl_id)
        phase_name = self._get_phase_name(phase)
        
        # Mức độ tắc
        if total > 15:
            status = "🔴DỰ ĐOÁN TẮC NGHIÊM TRỌNG"
        elif total > 10:
            status = "🟠Dự đoán tắc nặng"
        elif total > 5:
            status = "🟡Dự đoán tắc vừa"
        else:
            status = "🟢 Thông thoáng"
        
        logger.info(
            f"[{self.simulation_time:.1f}s] 🚦 {simple_name}: "
            f"{phase_name} ({duration:.0f}s) | "
            f"Đông-Tây: {ew_count} xe | Bắc-Nam: {ns_count} xe | "
            f"{status}"
        )
    
    # ===== Statistics =====
    def get_vehicle_statistics(self) -> Dict:
        """Lấy thống kê xe"""
        try:
            vehicle_ids = traci.vehicle.getIDList()
            
            stats = {
                'total_vehicles': len(vehicle_ids),
                'emergency_vehicles': 0,
                'avg_speed': 0.0,
                'vehicles_by_type': defaultdict(int)
            }
            
            total_speed = 0.0
            for vehicle_id in vehicle_ids:
                try:
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    vtype = traci.vehicle.getTypeID(vehicle_id)
                    
                    total_speed += speed
                    stats['vehicles_by_type'][vtype] += 1
                    
                    # Đếm xe cứu thương theo loại xe
                    if vtype == 'emergency':
                        stats['emergency_vehicles'] += 1
                
                except:
                    pass
            
            if vehicle_ids:
                stats['avg_speed'] = total_speed / len(vehicle_ids)
            
            return stats
        except:
            return {}
    
    def log_statistics(self):
        """Log thống kê hệ thống"""
        try:
            stats = self.get_vehicle_statistics()
            if not stats:
                return
            
            self.stats['total_vehicles'] = stats['total_vehicles']
            self.stats['emergency_vehicles'] = stats['emergency_vehicles']
            
            logger.info(
                f"\n{'='*70}\n"
                f"📊 THỐNG KÊ LÚC {self.simulation_time:.0f}s\n"
                f"{'='*70}\n"
                f"  🚗 Tổng xe: {stats['total_vehicles']} chiếc\n"
                f"  🚙 Ô tô: {stats['vehicles_by_type']['car']}\n"
                f"  🏍️  Xe máy: {stats['vehicles_by_type']['motorcycle']}\n"
                f"  🚑 Xe cứu thương: {stats['emergency_vehicles']} chiếc 🚑\n"
                f"  ⚡ Tốc độ TB: {stats['avg_speed']:.1f} m/s\n"
                f"  💬 Tin VANET: Gửi {self.stats['vanet_messages_sent']}, "
                f"Nhận {self.stats['vanet_messages_received']}\n"
                f"{'='*70}\n"
            )
        except Exception as e:
            logger.warning(f"Lỗi log thống kê: {e}")
    
    # ===== Main Simulation Loop =====
    def run_simulation(self, max_steps: int = 36000):
        """Chạy mô phỏng chính"""
        logger.info(f"\n{'='*70}")
        logger.info(f"🚀 Smart Traffic Control System - VANET")
        logger.info(f"⏱️  Bắt đầu: {datetime.now().strftime('%H:%M:%S')}")
        logger.info(f"{'='*70}\n")
        
        try:
            step = 0
            while step < max_steps:
                try:
                    traci.simulationStep()
                except:
                    logger.info("✅ Mô phỏng kết thúc")
                    break
                
                self.simulation_time = step * 0.1  # 0.1s per step
                
                # Thu thập beacon mỗi 0.5s
                if step % 5 == 0:
                    self.process_vehicle_beacons(self.simulation_time)
                
                # Điều khiển đèn
                for tl_id in self.traffic_lights:
                    traffic_info = self.analyze_traffic_at_junction(tl_id)
                    self.adjust_traffic_light(tl_id, traffic_info)
                
                # Log thống kê mỗi 100s
                if step % 1000 == 0 and step > 0:
                    self.log_statistics()
                
                step += 1
            
            # Báo cáo cuối
            logger.info(f"\n{'='*70}")
            logger.info(f"✅ Hoàn thành mô phỏng!")
            logger.info(f"⏱️  Tổng thời gian: {self.simulation_time:.1f}s")
            logger.info(f"{'='*70}\n")
            self.log_statistics()
            
        except KeyboardInterrupt:
            logger.info("❌ Dừng mô phỏng bởi người dùng")
        except Exception as e:
            logger.error(f"❌ Lỗi: {e}", exc_info=True)
        finally:
            self.stop_simulation()
    
    def stop_simulation(self):
        """Dừng SUMO"""
        try:
            traci.close()
            logger.info("✅ Đóng SUMO")
        except:
            pass


def main():
    """Hàm chính"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Smart Traffic System')
    parser.add_argument('--config', type=str, default='thuan.sumocfg')
    parser.add_argument('--simtime', type=float, default=3600)
    
    args = parser.parse_args()
    
    # Đường dẫn
    project_dir = os.path.dirname(os.path.abspath(__file__))
    sumo_config = os.path.join(project_dir, args.config)
    
    if not os.path.exists(sumo_config):
        logger.error(f"❌ Không tìm thấy file: {sumo_config}")
        sys.exit(1)
    
    # Tạo controller
    controller = SmartTrafficController(sumo_config)
    
    # Khởi động
    if not controller.start_simulation():
        sys.exit(1)
    
    # Chạy
    try:
        controller.run_simulation(max_steps=int(args.simtime * 10))
    except KeyboardInterrupt:
        logger.info("\n🛑 Dừng bởi người dùng")
    finally:
        controller.stop_simulation()


if __name__ == '__main__':
    main()