"""WASAPI loopback capture and low-rate mono PCM conversion for motor audio."""
import numpy as np
from PyQt5 import QtCore


class SystemAudioCapture(QtCore.QThread):
    pcm_block = QtCore.pyqtSignal(int, bytes)
    failed = QtCore.pyqtSignal(str)

    INPUT_RATE = 16000
    OUTPUT_RATE = 8000
    BLOCK_FRAMES = 48  # 3 ms at 16 kHz -> 24 samples at 8 kHz

    def __init__(self, parent=None):
        super().__init__(parent)
        self._seq = 0
        self._dc_x = 0.0
        self._dc_y = 0.0
        n = np.arange(-15, 16, dtype=np.float64)
        cutoff = 3600.0 / self.INPUT_RATE
        taps = 2.0 * cutoff * np.sinc(2.0 * cutoff * n) * np.hamming(n.size)
        self._fir = (taps / np.sum(taps)).astype(np.float32)
        self._fir_history = np.zeros(n.size - 1, dtype=np.float32)

    def convert_block(self, samples):
        samples = np.asarray(samples, dtype=np.float32)
        mono = np.mean(samples, axis=1, dtype=np.float32) if samples.ndim == 2 else samples.reshape(-1)
        if mono.size != self.BLOCK_FRAMES:
            raise ValueError("WASAPI 音频块帧数不正确")
        joined = np.concatenate((self._fir_history, mono))
        filtered = np.convolve(joined, self._fir, mode="valid")
        self._fir_history = joined[-(self._fir.size - 1):].copy()
        reduced = filtered[::2]
        pcm = np.empty(reduced.size, dtype=np.int8)
        for i, value in enumerate(reduced):
            y = float(value) - self._dc_x + 0.985 * self._dc_y
            self._dc_x, self._dc_y = float(value), y
            pcm[i] = np.int8(max(-127, min(127, round(y * 127.0))))
        return pcm.tobytes()

    def run(self):
        try:
            import soundcard as sc
            speaker = sc.default_speaker()
            if speaker is None:
                raise RuntimeError("Windows 没有可用的默认播放设备")
            loopback = sc.get_microphone(id=str(speaker.name), include_loopback=True)
            if loopback is None:
                raise RuntimeError("无法打开默认播放设备的 WASAPI 回环输入")
            with loopback.recorder(samplerate=self.INPUT_RATE, channels=2,
                                   blocksize=self.BLOCK_FRAMES) as recorder:
                while not self.isInterruptionRequested():
                    samples = recorder.record(numframes=self.BLOCK_FRAMES)
                    pcm = self.convert_block(samples)
                    self.pcm_block.emit(self._seq, pcm)
                    self._seq = (self._seq + 1) & 0xFFFF
        except Exception as exc:
            self.failed.emit(f"系统音频回环采集失败：{exc}")
