from TTS.api import TTS 
class VoiceGenerator:
    def __init__(self, model_name="tts_models/zh-CN/baker/tacotron2-DDC-GST"):
        self.tts = TTS(model_name=model_name)

    def generate_voice(self, text, file_path):
        self.tts.tts_to_file(text=text, file_path=file_path, progress_bar=False)
          
if __name__ == "__main__":
    from playsound import playsound
    voice_gen = VoiceGenerator()
    for text in ['101', '102', '103', '104', '105']:
        text = f"{text}房间已送达。"
        file_path = f"{text}.wav"
        voice_gen.generate_voice(text, file_path)
        print(f"语音已生成并保存到 {file_path}")

    
