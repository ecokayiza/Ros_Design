from TTS.api import TTS 
class VoiceGenerator:
    def __init__(self, model_name="tts_models/zh-CN/baker/tacotron2-DDC-GST"):
        self.tts = TTS(model_name=model_name)

    def generate_voice(self, text, file_path):
        self.tts.tts_to_file(text=text, file_path=file_path, progress_bar=False)
          
if __name__ == "__main__":
    from playsound import playsound
    voice_gen = VoiceGenerator()
    text = "我想要一个汉堡和一杯可乐。"
    file_path = "output1.wav"
    voice_gen.generate_voice(text, file_path)
    print(f"语音已生成并保存到 {file_path}")
    text = "我想要一个西红柿炒鸡蛋。"
    file_path = "output2.wav"
    voice_gen.generate_voice(text, file_path)
    print(f"语音已生成并保存到 {file_path}")
    playsound("output1.wav")
    playsound("output2.wav")
    
