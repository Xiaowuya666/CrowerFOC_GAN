import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from collections import deque
import threading
import unittest
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crowerfoc_protocol import MSG_AUDIO, crc8
from crowerfoc_transport import SerialWorker
from system_audio import SystemAudioCapture


class SystemAudioTests(unittest.TestCase):
    def test_capture_conversion_is_mono_8khz_signed_pcm(self):
        capture = SystemAudioCapture()
        t = np.arange(48, dtype=np.float32) / 16000.0
        stereo = np.column_stack((0.5 * np.sin(2 * np.pi * 400 * t),
                                  0.5 * np.sin(2 * np.pi * 400 * t)))
        pcm = capture.convert_block(stereo)
        self.assertEqual(len(pcm), 24)
        signed = np.frombuffer(pcm, dtype=np.int8)
        self.assertGreater(int(signed.max()), 0)
        self.assertLess(int(signed.min()), 0)

    def test_audio_block_frame_format_and_bounded_queue(self):
        worker = SerialWorker.__new__(SerialWorker)
        worker._stop = threading.Event()
        worker._lock = threading.Lock()
        worker._audio_active = True
        worker._audio_blocks = deque(maxlen=4)
        for seq in range(6):
            self.assertTrue(worker.queue_audio_block(seq, bytes([seq]) * 24))
        self.assertEqual(len(worker._audio_blocks), 4)
        frame = worker._audio_blocks[-1]
        self.assertEqual(frame[:3], bytes((0xAA, 0x55, MSG_AUDIO)))
        length = frame[3]
        payload = frame[4:4 + length]
        self.assertEqual(payload[:4], bytes((1, 5, 0, 24)))
        self.assertEqual(payload[4:], bytes([5]) * 24)
        self.assertEqual(frame[-1], crc8(frame[2:-1]))

    def test_audio_queue_rejects_oversized_blocks(self):
        worker = SerialWorker.__new__(SerialWorker)
        worker._stop = threading.Event()
        worker._lock = threading.Lock()
        worker._audio_active = True
        worker._audio_blocks = deque(maxlen=4)
        self.assertFalse(worker.queue_audio_block(0, bytes(56)))
        self.assertEqual(len(worker._audio_blocks), 0)


if __name__ == "__main__":
    unittest.main()
