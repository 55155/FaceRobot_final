# modules.py

import json
import sys
import os
import time
import winsound as sd
import random
import string
from playsound import playsound
import openai
import pyaudio
from gtts import gTTS
from pydub import AudioSegment
from six.moves import queue

from movement_module import *
from config import *

import multiprocessing
import re
pattern_1 = r"B: '(.*?)'"
pattern_2 = r"B:'(.*?)'"
pattern_3 = r"B : '(.*?)'"
playaudio = None

# API와 인증정보 설정
openai.api_key = OPENAI_API_KEY
key_path = CURRENT_PATH + GOOGLE_APPLICATION_CREDENTIALS
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = key_path


class GPTAssistant:
    def __init__(self, language="Kor", use_robot=False):
        self.start_time = time.time()
        self.time = time.time()
        self.language = language
        self.use_robot = use_robot
        self.first_start = True
        self.is_waiting = False
        self.status_path = CURRENT_PATH + "robot_status.txt"
        self.stt_lan_code, self.gtts_lang_code, self.start_ment, self.msg, self.conditions, self.responses, self.awake_robot= self.prepare_variables()

    def prepare_variables(self):
        with open(CURRENT_PATH + "ray_conversations.json", "r", encoding="utf-8") as f:
            msg_data = json.load(f)

        conv_data = msg_data.get(self.language)
        stt_lan_code = conv_data["stt_lan_code"]    # a BCP-47 language tag (http://g.co/cloud/speech/docs/languages)
        gtts_lang_code = conv_data["gtts_lang_code"]
        start_ment = conv_data["start_ment"]
        prepared_msg = conv_data["msg"]
        conditions = conv_data["conditions"]    # QUIT_PROGRAM, SING_A_SONG, INTRODUCE 등
        responses = conv_data["responses"]       # 분기에 따른 대답들(script)
        awake_robot = conv_data["AWAKE_ROBOT"]  # sleep모드에서 깨우기
        return (
            stt_lan_code,
            gtts_lang_code,
            start_ment,
            prepared_msg,
            conditions,
            responses,
            awake_robot
        )
    

    def chk_dir(self):
        if not os.path.exists(CURRENT_PATH + 'Audio'):
            os.makedirs(CURRENT_PATH + 'Audio')
        if self.use_robot :
            if not os.path.exists(CURRENT_PATH + 'Headmotion'):
                os.makedirs(CURRENT_PATH + 'Headmotion')
            if not os.path.exists(CURRENT_PATH + 'Mouthmotion'):
                os.makedirs(CURRENT_PATH + 'Mouthmotion')


    def chstatus(self,status):
        with open( self.status_path, "w") as f:
            f.write(status)

def worker(assistant, result_que):
    try:
        completion = openai.chat.completions.create(
            # model="gpt-3.5-turbo",
            model="gpt-4",
            messages=assistant.msg,
            temperature=0.5,
            max_tokens=200
        )
        result_que.put(completion)
    except openai.error.OpenAIError as e:
        result_que.put(e)


def safe_chat_completion_(assistant):
    result_que = multiprocessing.Queue()
    p = multiprocessing.Process(target=worker, args=(assistant, result_que))
    p.start()
    p.join(timeout=30)

    if p.is_alive():
        p.terminate()
        print("Request Timeout.")
        return None
    else:
        result = result_que.get()
        if isinstance(result, openai.error.OpenAIError):
            print(f"An error occurred: {result}")
            return None
        return result


# 대답 생성 및 출력 함수 정의
def _generate_answer(assistant):
    start_time = time.time()
    completion = openai.chat.completions.create(
        # model="gpt-3.5-turbo",
        model="gpt-4",
        messages=assistant.msg,
        temperature=0.5,
        max_tokens=200
    )
    # TimeOut 방지책
    # completion = safe_chat_completion_(assistant=assistant)

    # 시간 측정
    print(time.time() - start_time)

    if completion is None:
        generate_audio(assistant=assistant, text="아, 죄송해요 잠시 딴 생각 중이었어요. 다시 말씀해 주실래요?")
        assistant.msg.pop(1)
        return

    # answer_text = completion["choices"][0]["message"]["content"]
    answer_text = completion.choices[0].message.content
    
    while True :
        if any(char in string.punctuation for char in answer_text) :
            assistant.msg.append({"role": "assistant",
                        "content": answer_text})
            generate_audio(assistant, answer_text)
            break
        else: 
            print("Incomplete answer text. Pop more msgs.")
            assistant.msg.pop(1)
            assistant.msg.pop(2)
            time.sleep(0.04)


def append_msg(assistant, role, input_text):
    assistant.msg.append({"role": role,
                "content": input_text })


def generate_answer(assistant, input_text):
    while True:
        try :
            append_msg(assistant, "user", input_text=input_text)
            _generate_answer(assistant)
            return
        # except openai.error.InvalidRequestError:
        except openai.APIConnectionError:
        # 챗봇 API에 전송한 메시지가 최대 길이를 초과하는 경우
            assistant.msg.pop(1)
            assistant.msg.pop(2)
            time.sleep(0.04)
            print("Retry openai(msgLength)")
        # except openai.error.RateLimitError:
        except openai.RateLimitError:
            print("Retry openai(overloadRequests)")
            time.sleep(0.04)
            pass
        # except openai.error.APIError:
        except openai.APIError:
            print("retry openai(APIError)")
            time.sleep(0.04)
            pass


class MicrophoneStream(object):
    """
    Opens a recording stream as a generator yielding the audio chunks.
    """
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
            # 현재 1채널 오디오만 지원
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
            input_device_index= 1

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

    def get_read_available(self):
        return self._audio_stream.get_read_available()


def clear_buffer(stream):
    while stream.get_read_available() > 0:
        stream.read(stream.get_read_available()) 


def process_voice_input(assistant, responses):
    """
    STT를 통해 전달받은 음성 데이터를 처리하고 대답을 생성
    """
    # https://goo.gl/tjCPAU.
    # print only the transcription for the top alternative of the top result.
    for response in responses:
        if not response.results:
            continue

        # playaudio subprocess가 실행 중인데 사용자가 말을 끊을 시
        # 말하는 것을 멈춤
        if (playaudio is not None) and playaudio.is_alive():
            playaudio.terminate()
        
        # 터미널에 사용자 보이스 입력 표현
        sys.stdout.write("◾")
        sys.stdout.flush()            
        
        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # 최종 결과일 경우, 대답 생성    
        if result.is_final:
            transcript = result.alternatives[0].transcript
            sys.stdout.write("\n\n")
            print('[User] "' + transcript + '"', end = '\n\n')
            # 특정 단어에 따른 분기 처리
            for condition, keywords in assistant.conditions.items():
                transcript = transcript.lower()
                if all([keyword in transcript for keyword in keywords]):
                    generate_audio(assistant, assistant.responses[condition])
                    if condition == "SING_A_SONG" or condition == "QUIT_PROGRAM" : # 로봇의 동작에 영향을 끼치는 condition
                        assistant.chstatus(condition)
                    return
                
            generate_answer(assistant, transcript)
            return


def process_sleep_mode(assistant, responses):
    """
    STT를 통해 전달받은 음성 데이터를 처리하고 대답을 생성
    """
    # https://goo.gl/tjCPAU.
    # print only the transcription for the top alternative of the top result.
    for response in responses:
        if not response.results:
            continue
       
        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # 최종 결과일 경우, 대답 생성    
        if result.is_final:
            transcript = result.alternatives[0].transcript

            # 특정 단어에 따른 분기 처리
            transcript = transcript.lower()
            if all([keyword in transcript for keyword in assistant.awake_robot["condition"]]):
                sys.stdout.write("\n\n")
                # print('[User] "' + transcript + '"', end = '\n\n')
                awake_ment = random.choice(assistant.awake_robot["response"])
                generate_audio(assistant, awake_ment)
                if assistant.language == "Kor" : append_msg(assistant, "user", "레이야 뭐해?")
                elif assistant.language == "En" : append_msg(assistant, "user", "What's up, Ray?")
                append_msg(assistant, "assistant", awake_ment)
                # print(assistant.msg)
            return


# def generate_audio(assistant, text):
#     """
#     텍스트를 통해 오디오를 생성
#     로봇이 있을 경우 해당 오디오에 대한 모션 생성
#     """
#     print('[RAY] "' + text + '"', end='\n\n')
#     file_name = 'AIspeaker_answer'
#     mp3_file = audio_file_path(file_name, 'mp3')
#     wav_file = audio_file_path(file_name, 'wav')
#     num = 1
#     while True: 
#         try :
#             tts = gTTS(text=text, lang=assistant.gtts_lang_code)
#             break
#         except :
#             print(f"gTTS ERROR. Retry #{num}..")
#             num+=1
#             time.sleep(0.1)
#             if num > 10 : 
#                 assistant.chstatus("QUIT_PROGRAM")
#                 return
#     tts.save(mp3_file)

#     if assistant.use_robot:
#         # convert to wav
#         sound = AudioSegment.from_mp3(mp3_file)
#         sound.export(wav_file, format="wav")
#         if generation_head_mouth_motion(file_name):
#             assistant.chstatus("READY_TO_SPEAK")
#         else : print("ERROR : could not generate movement")
#     elif not assistant.use_robot:
#         playsound(mp3_file)
#         if os.path.exists(mp3_file):  # remove file
#             os.remove(mp3_file)
#         assistant.chstatus("WAITING_FOR_USER_INPUT")

def generate_audio(assistant, text):
    """
    텍스트를 통해 오디오를 생성
    로봇이 있을 경우 해당 오디오에 대한 모션 생성
    """
    print('[RAY] "' + text + '"', end='\n\n')
    file_name = 'AIspeaker_answer'
    mp3_file = audio_file_path(file_name, 'mp3')
    wav_file = audio_file_path(file_name, 'wav')
    num = 1
    while True: 
        try :
            tts = gTTS(text=text, lang=assistant.gtts_lang_code)
            break
        except :
            print(f"gTTS ERROR. Retry #{num}..")
            num+=1
            time.sleep(0.1)
            if num > 10 : 
                assistant.chstatus("QUIT_PROGRAM")
                return
    tts.save(mp3_file)

    global playaudio

    if assistant.use_robot:
        sound = AudioSegment.from_mp3(mp3_file)
        sound.export(wav_file, format="wav")
        if generation_head_mouth_motion(file_name):
            assistant.chstatus("READY_TO_SPEAK")
        else : print("ERROR : could not generate movement")
    elif not assistant.use_robot:
        playaudio = multiprocessing.Process(target=play_audio, args=(assistant, mp3_file, wav_file, file_name))
        playaudio.start()



def play_audio(assistant, mp3_file, wav_file, file_name):
    if assistant.use_robot:
        # convert to wav
        sound = AudioSegment.from_mp3(mp3_file)
        sound.export(wav_file, format="wav")
        if generation_head_mouth_motion(file_name):
            assistant.chstatus("READY_TO_SPEAK")
        else : print("ERROR : could not generate movement")
    elif not assistant.use_robot:
        playsound(mp3_file)
        if os.path.exists(mp3_file):  # remove file
            os.remove(mp3_file)
        assistant.chstatus("WAITING_FOR_USER_INPUT")


def audio_file_path(file_name, extension):
    audio_file_path = CURRENT_PATH + 'Audio/' + file_name + '.' + extension
    return audio_file_path


def beep_sound():
    FREQUENCY = 1000    # range : 37 ~ 32767
    DURATION = 400     # ms
    sd.Beep(FREQUENCY, DURATION)


def elapsed_time(start_time):
    elapsed_time = int(time.time()-start_time)
    h = elapsed_time // 3600
    min = (elapsed_time % 3600) // 60
    s = elapsed_time % 60
    return h,min,s



# file_name = 'COMPLETE_SINGING_ko'
# text = '잘 감상하셨나요? 노래가 듣고 싶다면 언제든지 말해 주세요'    
# mp3_file = CURRENT_PATH + 'Audio/'+file_name+'.mp3'
# wav_file = CURRENT_PATH + 'Audio/'+file_name+'.wav'
# tts = gTTS(text=text, lang='ko')
# tts.save(mp3_file)
# sound = AudioSegment.from_mp3(mp3_file)
# sound.export(wav_file, format="wav")
# movement_module.generation_head_mouth_motion(file_name)