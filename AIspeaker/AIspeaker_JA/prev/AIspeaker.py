import openai,json, pprint
import time, os
import speech_recognition as sr
from gtts import gTTS
from playsound import playsound
import tkinter
import __movementFunc
from pydub import AudioSegment

path = 'C:/Users/user/Desktop/FaceRobot/OpenFace-master/x64/Release/AIspeaker/'
ready2listen = False
#음성인식(듣기)
def callback(recognizer, audio):
    with open( path + "data_sync.txt", "r") as f:
        num = int(f.read())
    global ready2listen

    #Prevent to listen computer's audio
    if (num == 0) & (ready2listen==False):
        ready2listen = True
        print("듣고 있어요")
        playsound(path + "dingdong.mp3")

    elif ready2listen :
        print("음성 인식 완료")
        try:
            text = recognizer.recognize_google(audio, language='ko') #하루에 50회 제한
            print('[사용자] '+ text)
            answer(text)
            global prev_input
            prev_input = text
            ready2listen = False
        except sr.UnknownValueError:
            print('인식 실패') # 음성 인식 실패한 경우
        except sr.RequestError as e:
            print('요청 실패 : {0}'.format(e)) #API Key 오류, 네트워크 단절 등


#대답
def answer(input_text):
    global stop_listening
    if ('중지' in input_text) or ('종료' in input_text):
        answer_text = "알겠어요. 그럼 저는 잠시 쉬고 있을게요."
        stop_listening(wait_for_stop=False)
        head_mouth_generation(answer_text)
        global flag
        flag = False
        with open(path + "data_sync.txt", "w") as f:
            f.write("-1")
            
        return
    # elif '이름' in input_text:
    #     my_info = input_text
    else:
        global prev_answer
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                # {
                #     "role": "system",
                #     "content" : my_info,
                # },
                {
                    "role": "system",
                    "content" : "Speak Korean. Do smalltalk.",
                },
                {
                    "role": "user", 
                    "content": prev_input
                },
                {
                    "role": "assistant", 
                    "content": prev_answer
                },
                {
                    "role": "user",
                    "content": input_text,
                }
            ],
        )
        answer_text = completion["choices"][0]["message"]["content"]
        prev_answer = answer_text
    head_mouth_generation(answer_text)
    
# 소리내어 읽기 (TTS)
def speak(text):
    print('[인공지능] ' + text)
    file_name = 'AIspeaker_answer.mp3'
    tts = gTTS(text=text, lang='ko')
    tts.save(path + file_name)
    playsound(path + file_name)
    if os.path.exists(path + file_name): # 파일 삭제
        os.remove(path + file_name)

def head_mouth_generation(text):
    print('[인공지능] ' + text)
    file_name = 'AIspeaker_answer'
    tts = gTTS(text=text, lang='ko')
    tts.save(path + file_name + '.mp3')

    # convert to wav
    sound = AudioSegment.from_mp3(path + file_name + ".mp3")
    sound.export(path + file_name + ".wav", format="wav")

    if __movementFunc.generation_head_mouth_motion(file_name):
        with open(path + "data_sync.txt", "w") as f:
            f.write("1")
    if os.path.exists( path + file_name + '.mp3'): # 파일 삭제
        os.remove(path + file_name + '.mp3')
    
        
def start():
    head_mouth_generation("무엇을 도와드릴까요?")
    global stop_listening
    stop_listening = r.listen_in_background(m, callback)
    # 프로그램 무한반복
    while flag:
        # print('asdfasdf')
        time.sleep(0.1)
    

# 창닫기 함수
# def exit_window(): window.destroy()

############################################################################################
    
openai.api_key = "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"
iter = 0
flag=True

prev_input= "안녕하세요"
prev_answer= "안녕하세요. 만나서 반가워요"

r = sr.Recognizer()
m = sr.Microphone()

###############################################################

# #button 만들기

# # Tk Class 선언으로 window 창 생성
# window = tkinter.Tk()

# # 생성할 window 창의 크기 및 초기 위치 설정 매서드: geometry()
# window_width = 400
# window_height = 200
# window_pos_x = 700
# window_pos_y = 100

# window.geometry("{}x{}+{}+{}".format(window_width, window_height, window_pos_x, window_pos_y))

# # 생성한 Window 창의 크기 조절 가능 여부 설정: resizable()
# window.resizable(False, False)   # True, False 대신 1, 0을 사용할 수 있음

# # 생성한 Window 창의 Title 설정: title()
# window.title("Chatting Robot")

# button_start = tkinter.Button(window, text="Start", width=20, height=3, command=lambda: start())
# button_start.place(x=130, y=30)
# button_exit = tkinter.Button(window, text="종료", width=20, height=3, command=exit_window)
# button_exit.place(x=130, y=100)

# # 생성한 창을 유지하기 위한 코드 작성
# window.mainloop()