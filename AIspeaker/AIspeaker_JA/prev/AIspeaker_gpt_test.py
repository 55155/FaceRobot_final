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
from google.api_core.exceptions import DeadlineExceeded
from google.api_core.exceptions import Unknown
import re
import time
import winsound as sd
import __movementFunc


RAY = True
language = "Kor" #Kor/En
first_start = True

path = 'C:/Users/user/Desktop/FaceRobot/OpenFace-master/x64/Release/AIspeaker/'
openai.api_key = "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"
key_path = path + "aispeaker-383000-16cb064b4449.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = key_path
credentials = service_account.Credentials.from_service_account_file(key_path)
time1 = time.time()

if language == "Kor" :
    msg=[
            {
                "role": "system",
                "content" : "당신은 고정된 플랫폼에 머리만 있는 애니메트로닉스 로봇 RAY입니다. RAY는 매우 친절하고 똑똑하며 노래도 부를 줄 알지만 날씨와 시간 같은 실시간 정보는 모릅니다. RAY는 한국에 살고 있지만, 스웨덴을 정말 가보고 싶어합니다. 사용자의 메세지에 두 문장 이내로 짧게 대답하세요"
            }
    ]
elif language == "En":
        msg=[
            {
                "role": "system",
                "content" : "The following is a conversation with Ray, animatronics robot with only a head on a fixed platform. Ray is very kind and smart but doesn't know the real-time information like weather and time. Ray lives in Korea, but Ray thinks Sweden is a country that Ray really wants to visit."
            }
    ]

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
    
    for response in responses:
        if not response.results:
            continue
        else:
            with open( path + "data_sync.txt", "r") as f:
                status = f.read()
            if RAY :
                if "WAITING_FOR_USER_INPUT" not in status:        #레이 있을 때
                    continue 
            elif not RAY :
                global time1
                if time.time()-time1 > 4 :                          #레이 없을 때
                    with open(path + "data_sync.txt", "w") as f:
                        f.write("WAITING_FOR_USER_INPUT")
              
        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript
                
        if result.is_final:
            if not RAY : 
                if "WAITING_FOR_USER_INPUT" not in status:                #레이 없을 때
                    time.sleep(0.5)
                    with open(path + "data_sync.txt", "w") as f:
                        f.write("WAITING_FOR_USER_INPUT")
                    beepsound()
                    continue 
            if "WAITING_FOR_USER_INPUT" in status :
                if language == "Kor" : 
                    exit_condition = '종료' in transcript or '중지' in transcript
                    exit_ment = "알겠어요. 그럼 저는 잠시 쉬고 있을게요."
                    song_condition = "노래" in transcript and "불러" in transcript
                    song_ment = "알겠어요. 제가 준비한 노래를 들려드릴게요!"
                    introduce_condition = "자기" in transcript and "소개" in transcript
                    introduce_ment = "스웨덴에서 오신 존경하는 손님 여러분 안녕하세요. 저는 오디오 에니메트로닉 로봇, RAY에요. 저는 2022년 한국과학기술연구원에서 태어났어요. 임세혁박사님과 그의 팀이 저를 만들었답니다."
                elif language == "En" :
                    exit_condition = re.search(r"\b(bye)\b", transcript, re.I)
                    exit_ment = "알겠어요. 그럼 저는 잠시 쉬고 있을게요."
                    song_condition = 'sing a song' in transcript.lower()
                    song_ment =  "Sure. Please enjoy my performance!"
                    introduce_condition = 'introduce yourself' in transcript.lower()
                    introduce_ment = 'Hello, my honored guests from Sweden. I am an audio-animatronic robot, ray. I was born in 2022 at Korea Institute of Science and Technology in South Korea. Dr. Sehyuk Yim and his team created me.'
                if exit_condition:
                    print("\n[User] " + transcript)

                    if RAY:
                        head_mouth_generation(exit_ment)
                    elif not RAY :
                        speak(exit_ment)

                    with open(path + "data_sync.txt", "w") as f:
                        f.write("QUIT_PROGRAM")
                    break

                elif song_condition:
                    print("\n[User] " + transcript)
                       
                    if RAY:
                        head_mouth_generation(song_ment)
                        with open(path + "data_sync.txt", "w") as f:
                            f.write("SING_A_SONG")
                    elif not RAY :
                        speak(song_ment)                        

                elif introduce_condition : 
                    print("\n[User] " + transcript)
                        
                    if RAY :
                        head_mouth_generation(introduce_ment)
                    elif not RAY :
                        speak(introduce_ment)

                elif 'what can you do' in transcript.lower() : 
                    print("\n[User] " + transcript)
                    answer_text = 'I have audio-based motion intelligence. I can act like a human when speaking or singing. I also have conversational AI. That is based on the text completion model, not chatGPT.'
                        
                    if RAY :
                        head_mouth_generation(answer_text)
                    elif not RAY :
                        speak(answer_text)

                else:
                    print("\n[User] " + transcript)
                    answer(transcript)


# 대답 생성 및 출력 함수 정의
def generate_answer(input_text):
    global msg
    
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=msg,
    )
    answer_text = completion["choices"][0]["message"]["content"]
    answer_text.replace( '\n' , '' )
    msg.append({"role": "assistant",
                      "content": answer_text})
    
    if RAY :
        head_mouth_generation(answer_text)
    elif not RAY :
        speak(answer_text)


def answer(input_text):
    global msg

    try :
        msg.append({"role": "user",
            "content": input_text +"두문장 이내로 대답해줘"})
        generate_answer(input_text)
    except openai.error.InvalidRequestError:
    # 챗봇 API에 전송한 메시지가 최대 길이를 초과하는 경우
        msg.pop(0)
        generate_answer(input_text)

# 소리내어 읽기 (TTS)
def speak(text):
    print('\n[RAY] ' + text)
    file_name = 'AIspeaker_answer'
    if language == "Kor" :
        lang_code = 'ko'
    elif language == "En":
        lang_code = 'en'
        
    tts = gTTS(text=text, lang= lang_code)

    if not os.path.exists(path + 'Audio'):
        os.makedirs(path + 'Audio')
    tts.save(path + 'Audio/'+ file_name + '.mp3')

    with open(path + "data_sync.txt", "w") as f:
        f.write("READY_TO_SPEAK")
    playsound(path +'Audio/'+ file_name + '.mp3')
    global time1
    time1 = time.time()
    if os.path.exists(path +'Audio/'+ file_name + '.mp3'): # 파일 삭제
        os.remove(path +'Audio/'+ file_name+ '.mp3')

def head_mouth_generation(text):
    print('\n[RAY] ' + text)
    file_name = 'AIspeaker_answer'
    if language == "Kor" :
        lang_code = 'ko'
    elif language == "En":
        lang_code = 'en'

    tts = gTTS(text=text, lang= lang_code)

    if not os.path.exists(path + 'Audio'):
        os.makedirs(path + 'Audio')
    tts.save(path + 'Audio/'+ file_name + '.mp3')

    # convert to wav
    sound = AudioSegment.from_mp3(path +'Audio/'+ file_name + '.mp3')
    sound.export(path +'Audio/'+ file_name + ".wav", format="wav")
    
    if os.path.exists(path +'Audio/'+ file_name + '.mp3'): # 파일 삭제
        os.remove(path +'Audio/'+ file_name + '.mp3')

    if __movementFunc.generation_head_mouth_motion(file_name):
        with open(path + "data_sync.txt", "w") as f:
            f.write("READY_TO_SPEAK")
    
# 경보음
def beepsound():
    fr = 1000    # range : 37 ~ 32767
    du = 400     # 1000 ms ==1second
    sd.Beep(fr, du) # winsound.Beep(frequency, duration)

def main():
    
    if language == "Kor" :
        start_ment = "안녕하세요! 저는 애니메트로닉스 로봇 RAY에요."
        # See http://g.co/cloud/speech/docs/languages

        language_code = "ko-KR" # a BCP-47 language tag
    elif language == "En" :
        start_ment = "Hello! I'm RAY, an animatronics robot."
        language_code = "en-US" # a BCP-47 language tag
    global first_start

    if not RAY and not first_start :                                              #레이 없을 때
        with open(path + "data_sync.txt", "w") as f:
            f.write("READY_TO_SPEAK")                  
        beepsound()
        
    if first_start:
        with open(path + "data_sync.txt", "w") as f:
            f.write("PYTHON_START")
        if RAY:
            head_mouth_generation(start_ment)
        elif not RAY :
            speak(start_ment)

        first_start = False


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
        try :
            global responses
            responses = client.streaming_recognize(streaming_config, requests, timeout=max_duration)
        except Unknown :
            print("no signal")
            main()

        # Now, put the transcription responses to use.
        try :
            listen_print_loop(responses)
        except DeadlineExceeded as e:
            print("Restart")
            main()
        

if __name__ == "__main__":
    main()