import yaml
from typing import List, Dict, Any

class ScheduleStore:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        self.schedule: List[Dict[str, Any]] = data.get('schedule', [])
        self.waypoints: List[List[float]] = data.get('waypoints', [])
        self._validate()

    def _validate(self):
        # 최소 검증: 타입/필드 체크
        assert isinstance(self.schedule, list)
        assert isinstance(self.waypoints, list)
        for wp in self.waypoints:
            assert isinstance(wp, list) and len(wp) == 2, "waypoint must be [lat, lon]"

    def get_schedule(self) -> List[Dict[str, Any]]:
        return self.schedule

    def get_waypoints(self) -> List[List[float]]:
        return self.waypoints
