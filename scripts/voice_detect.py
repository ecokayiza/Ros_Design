import os
import wave
import json
from vosk import Model, KaldiRecognizer

py_folder = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(py_folder, "checkpoints", "vosk-model-small-cn-0.22")
class VoiceDetector:
    def __init__(self, model_path):
        self.model = Model(model_path)
        self.recognizer = None

    def load_audio(self, audio_file):
        wf = wave.open(audio_file, "rb")
        self.recognizer = KaldiRecognizer(self.model, wf.getframerate())
        return wf

    def recognize(self, audio_file):
        wf = self.load_audio(audio_file)
        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break
            self.recognizer.AcceptWaveform(data)

        res = json.loads(self.recognizer.FinalResult())
        return res.get("text", "")
        
if __name__ == "__main__":
    audio_file = "/home/para/catkin_ws/src/ros_design/ttsmaker-file-2025-6-19-11-47-22.wav"
    detector = VoiceDetector(model_path)
    res1 = detector.recognize(audio_file)
    print("识别结果:", res1)
    
