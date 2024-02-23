from __future__ import division
from google.cloud import speech
import pyaudio
from pydub import AudioSegment
from gtts import gTTS
from playsound import playsound
import openai
from six.moves import queue
import os
from google.oauth2 import service_account
import __movementFunc
from google.api_core.exceptions import DeadlineExceeded
import re


path = 'C:/Users/user/Desktop/FaceRobot/OpenFace-master/x64/Release/AIspeaker/'

openai.api_key = "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"
key_path = path + "aispeaker-383000-16cb064b4449.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = key_path
credentials = service_account.Credentials.from_service_account_file(key_path)


first_start = True
prompt_text = None

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)

def listen_print_loop(responses):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    try:
        for response in responses:
            if not response.results:
                continue
            else:
                with open( path + "data_sync.txt", "r") as f:
                    status = f.read()
                if "WAITING_FOR_USER_INPUT" not in status:
                     continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript
                
            if result.is_final:
                if "WAITING_FOR_USER_INPUT" in status :
                   
#                    if '종료' in transcript or '중지' in transcript:
                    if re.search(r"\b(exit|quit|stop)\b", transcript, re.I):
#                        answer_text = "알겠어요. 그럼 저는 잠시 쉬고 있을게요."
                        answer_text = "Okay. I'll take a break. See you again!"
                        head_mouth_generation(answer_text)
                        with open(path + "data_sync.txt", "w") as f:
                            f.write("QUIT_PROGRAM")
                        break

                    elif 'sing a song' in transcript.lower() or 'sing another song' in transcript.lower():
                        print("\n[User] " + transcript)
#                        answer_text = "알겠어요. 제가 준비한 노래를 들려드릴게요!"
                        answer_text = "Sure. Please enjoy my performance!"
                        head_mouth_generation(answer_text)
                        global prompt_text
#                        prompt_text = prompt_text +  "친구:어때요? 잘 감상하셨나요? 노래가 듣고 싶다면 언제든지 얘기해도 좋아요.\n당신:"
                        prompt_text = prompt_text +  "Friend:How was it? Did you enjoy it? You can tell me whenever you want to listen to the song.\nYou:"
                        with open(path + "data_sync.txt", "w") as f:
                            f.write("SING_A_SONG")
                    elif 'introduce yourself' in transcript.lower() : 
                        answer_text = 'Hello, my honored guests from Sweden. I am an audio-animatronic robot, ray. I was born in 2022 at Korea Institute of Science and Technology in South Korea. Dr. Sehyuk Yim and his team created me.'
                        head_mouth_generation(answer_text)
                    elif 'what can you do' in transcript.lower() : 
                        answer_text = 'I have audio-based motion intelligence. I can act like a human when speaking or singing. I also have conversational AI. That is based on the text completion model, not chatGPT.'
                        head_mouth_generation(answer_text)
                    else:
                        print("\n[User] " + transcript)
                        answer(transcript)
 
    except DeadlineExceeded as e:
        print("Restart")
        main()

#대답
def answer(input_text):
    
    global prompt_text
#    prompt_text = prompt_text + "당신: " + input_text +"\친구: "
    prompt_text = prompt_text + "You:" + input_text +"\Friend:"
    response = openai.Completion.create(
        model="text-davinci-003",
        prompt=prompt_text,
        temperature=0.8,
        max_tokens=200,
        top_p=1.0,
        frequency_penalty=0.5,
        presence_penalty=0,
        stop=["You:"]
    )
    answer_text = response["choices"][0]["text"]
    answer_text.replace( '\n' , '' )
    prompt_text = prompt_text + answer_text + "\n"
    head_mouth_generation(answer_text)

# 소리내어 읽기 (TTS)
def speak(text):
    print('[RAY] ' + text)
    file_name = 'AIspeaker_answer.mp3'
    tts = gTTS(text=text, lang='ko')
    tts.save(path + file_name)
    playsound(path +'Audio/'+ file_name + '.mp3')
    if os.path.exists(path +'Audio/'+ file_name + '.mp3'): # 파일 삭제
        os.remove(path + file_name)
    with open(path + "data_sync.txt", "w") as f:
        f.write("WAITING_FOR_USER_INPUT")

def head_mouth_generation(text):
    print('\n[RAY] ' + text)
    file_name = 'AIspeaker_answer'
#    tts = gTTS(text=text, lang='ko')
    tts = gTTS(text=text, lang='en')
    tts.save(path +'Audio/'+ file_name + '.mp3')

    # convert to wav
    sound = AudioSegment.from_mp3(path +'Audio/'+ file_name + '.mp3')
    sound.export(path +'Audio/'+ file_name + ".wav", format="wav")

    if __movementFunc.generation_head_mouth_motion(file_name):
        with open(path + "data_sync.txt", "w") as f:
            f.write("READY_TO_SPEAK")
    if os.path.exists(path +'Audio/'+ file_name + '.mp3'): # 파일 삭제
        os.remove(path +'Audio/'+ file_name + '.mp3')

    # #for test    
    # with open(path + "data_sync.txt", "w") as f:
    #     f.write("0")

def main():
    global first_start
    if first_start:
#        head_mouth_generation("안녕하세요! 저는 애니메트로닉스 로봇 RAY에요.")
        head_mouth_generation("Hello! I'm RAY, an animatronics robot.")
        
        global prompt_text
#        prompt_text="당신: 네 이름이 뭐야?\n친구: 안녕하세요! 저는 애니메트로닉스 로봇 RAY에요.\n당신: 나랑 대화하자!\n친구: 좋아요. 저와 재미있게 놀아요."
        prompt_text=""""
        The following is a conversation with Ray, animatronics robot with only a head on a fixed platform. Ray is very kind and smart but doesn't know the real-time information like weather and time. Ray lives in Korea, but Ray thinks Sweden is a country that Ray really wants to visit.:

        You: What's your name?
        Friend: Hello! I'm RAY, an animatronics robot.
        You: Talk to me!
        Friend: All right. Have fun with me."
        """
        first_start = False

    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
#    language_code = "ko-KR"  # a BCP-47 language tag
    language_code = "en-US"  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=True
    )

    with MicrophoneStream(RATE, CHUNK) as stream:

        audio_generator = stream.generator()
        requests = (
            speech.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )

        max_duration = 300
        responses = client.streaming_recognize(streaming_config, requests, timeout=max_duration)

        # Now, put the transcription responses to use.
        listen_print_loop(responses)
        

# if __name__ == "__main__":
#     main()