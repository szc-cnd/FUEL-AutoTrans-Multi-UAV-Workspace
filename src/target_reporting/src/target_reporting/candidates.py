import math


class CandidateTracker:
    def __init__(self, distance_m=0.30, confirm_hits=3):
        self.distance_m = float(distance_m)
        self.confirm_hits = max(1, int(confirm_hits))
        self._items = []
        self._next = {}

    @staticmethod
    def _result_key(result):
        return tuple(sorted((str(k), str(v)) for k, v in result.items()))

    def update(self, target_type, result, position):
        key = self._result_key(result)
        best = None
        best_distance = None
        for item in self._items:
            if item["target_type"] != target_type or item["result_key"] != key:
                continue
            distance = math.sqrt(sum((float(position[a]) - item["position"][a]) ** 2 for a in ("x", "y", "z")))
            if distance <= self.distance_m and (best_distance is None or distance < best_distance):
                best, best_distance = item, distance
        if best is None:
            number = self._next.get(target_type, 0) + 1
            self._next[target_type] = number
            best = {
                "target_type": target_type,
                "result_key": key,
                "position": dict(position),
                "hits": 0,
                "reported": False,
                "local_number": number,
            }
            self._items.append(best)
        best["hits"] += 1
        alpha = 1.0 / best["hits"]
        for axis in ("x", "y", "z"):
            best["position"][axis] += alpha * (float(position[axis]) - best["position"][axis])
        newly_confirmed = best["hits"] >= self.confirm_hits and not best["reported"]
        if newly_confirmed:
            best["reported"] = True
        return best, newly_confirmed

