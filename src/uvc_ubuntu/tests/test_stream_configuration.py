import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class StreamConfigurationTest(unittest.TestCase):
    def test_formal_source_uses_yuv_only_384x288_without_header_offset(self):
        sensor = (ROOT / "hik_sensor.c").read_text(encoding="utf-8")
        camera = (ROOT / "uvc_camera.c").read_text(encoding="utf-8")

        self.assertIn("#define STREAM_TYPE_YUV_ONLY      10", sensor)
        self.assertIn(
            "stream_type_config(u, STREAM_TYPE_YUV_ONLY, 20)", sensor
        )
        self.assertIn("height = 288;", camera)
        self.assertIn("u->len = 384*288*2;", camera)
        self.assertIn("u->offset = offset_fix;", camera)
        self.assertNotIn("height = 292;", camera)
        self.assertNotIn("384*4*2", camera)
        self.assertIn("req_width=%d,req_height=%d", camera)


if __name__ == "__main__":
    unittest.main()
