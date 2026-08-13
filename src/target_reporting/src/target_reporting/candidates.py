import math


class CandidateTracker:
    # Detector geometry changes every frame and must not create a new logical
    # target.  Only semantic identity fields participate in de-duplication.
    _GEOMETRY_KEYS = {
        "points", "bbox", "center_u", "center_v", "u", "v", "cx", "cy",
        "depth", "area", "side_px", "score", "stable_count", "reason",
        "point_camera", "real_width", "real_height", "valid_depth_ratio",
        "depth_std", "pixel_area", "fill_ratio", "method", "preprocess",
        "validated", "confirmable",
        "detector_stable", "detector_confirmable",
    }

    def __init__(
        self,
        distance_m=0.30,
        confirm_hits=1,
        confirm_hits_by_type=None,
        confirmation_max_gap_s_by_type=None,
    ):
        self.distance_m = float(distance_m)
        self.confirm_hits = max(1, int(confirm_hits))
        self.confirm_hits_by_type = {
            str(key): max(1, int(value))
            for key, value in (confirm_hits_by_type or {}).items()
        }
        self.confirmation_max_gap_s_by_type = {
            str(key): max(0.0, float(value))
            for key, value in (confirmation_max_gap_s_by_type or {}).items()
        }
        self._items = []
        self._next = {}

    @staticmethod
    def _result_key(result):
        return tuple(
            sorted(
                (str(k), str(v))
                for k, v in result.items()
                if k not in CandidateTracker._GEOMETRY_KEYS
            )
        )

    def update(
        self,
        target_type,
        result,
        position,
        allow_confirmation=True,
        timestamp=None,
    ):
        key = self._result_key(result)
        qr_content = ""
        if target_type == "qr_code":
            qr_content = str(result.get("content", "") or "").strip()
        best = None
        best_distance = None
        for item in self._items:
            if item["target_type"] != target_type:
                continue
            same_decoded_qr = bool(
                qr_content
                and item.get("qr_content", "") == qr_content
            )
            if item["result_key"] != key and not same_decoded_qr:
                continue
            distance = math.sqrt(sum((float(position[a]) - item["position"][a]) ** 2 for a in ("x", "y", "z")))
            # A non-empty decoded QR payload is a stronger identity than its
            # drifting world coordinate.  Once registered, the same payload
            # must not be uploaded again after a FAST-LIO/TF position jump.
            within_identity_gate = same_decoded_qr or distance <= self.distance_m
            if within_identity_gate and (best_distance is None or distance < best_distance):
                best, best_distance = item, distance
        if best is None:
            number = self._next.get(target_type, 0) + 1
            self._next[target_type] = number
            best = {
                "target_type": target_type,
                "result_key": key,
                "qr_content": qr_content,
                "position": dict(position),
                "hits": 0,
                "last_confirmation_stamp": None,
                "reported": False,
                "local_number": number,
            }
            self._items.append(best)
        if not allow_confirmation:
            # Keep the candidate position available for RViz, but do not let
            # unvalidated observations accumulate toward a confirmation.
            if not best["reported"]:
                best["hits"] = 0
                best["last_confirmation_stamp"] = None
            for axis in ("x", "y", "z"):
                best["position"][axis] = float(position[axis])
            return best, False

        current_stamp = None if timestamp is None else float(timestamp)
        max_gap = self.confirmation_max_gap_s_by_type.get(target_type)
        last_stamp = best.get("last_confirmation_stamp")
        if (
            max_gap is not None
            and current_stamp is not None
            and last_stamp is not None
            and (current_stamp < last_stamp or current_stamp - last_stamp > max_gap)
        ):
            best["hits"] = 0

        best["hits"] += 1
        if current_stamp is not None:
            best["last_confirmation_stamp"] = current_stamp
        alpha = 1.0 / best["hits"]
        for axis in ("x", "y", "z"):
            best["position"][axis] += alpha * (float(position[axis]) - best["position"][axis])
        required_hits = self.confirm_hits_by_type.get(
            target_type, self.confirm_hits
        )
        newly_confirmed = best["hits"] >= required_hits and not best["reported"]
        if newly_confirmed:
            best["reported"] = True
        return best, newly_confirmed
