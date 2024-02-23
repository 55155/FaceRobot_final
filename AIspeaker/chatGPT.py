# main.py

import time
from google.cloud import speech
from google.api_core.exceptions import DeadlineExceeded
from modules import *
# anaconda
# current version: 22.9.0
def _main(assistant):

    if not assistant.use_robot and not assistant.first_start :  #레이 없을 때
        assistant.chstatus("WAITING_FOR_USER_INPUT")            
        beep_sound()

    if assistant.first_start:
        assistant.chstatus("PYTHON_START")
        assistant.chk_dir()
        generate_audio(assistant, assistant.start_ment)
        assistant.first_start = False

    
    # 오디오 녹음 매개변수 설정
    RATE = 16000
    CHUNK = int(RATE / 10)  # 100ms
    
    client = speech.SpeechClient()
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=assistant.stt_lan_code,
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=True
    )
    while True:
    
        with MicrophoneStream(RATE, CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )

            # 로봇 상태 읽기
            with open( assistant.status_path, "r") as f:
                status = f.read()
                print("length of status : ",len(status))

            if "QUIT_PROGRAM" in status: 
                break

            if "WAITING_FOR_USER_INPUT" not in status and "SLEEP_MODE" not in status:
                time.sleep(0.04)
                continue

            if "WAITING_FOR_USER_INPUT" in status:
                assistant.time = time.time()

            MAX_DURATION = 300
            MAX_RETRIES = 30
            RETRY_DELAY = 1 #second
            num_retries = 0
            
            while True:
                try:
                    # StreamingRecognize 메소드 호출
                    responses = client.streaming_recognize(
                        streaming_config, requests, timeout=MAX_DURATION
                    )
                    break  # 성공하면 while 루프를 빠져나감
                except Exception as e: # 아무 신호 없이 timeout이 발생시 오류 발생
                    num_retries += 1
                    h,min, s = elapsed_time(assistant.start_time)
                    print(f"Elapsed time : {h}h {min}min {s}s")
                    print(f"Error : {e} ")
                    print(f"Retry #{num_retries} in {RETRY_DELAY} seconds", end= "\n\n")
                    time.sleep(RETRY_DELAY) #s
                    if num_retries >= MAX_RETRIES:
                        raise
            
            patience_time = 300 # 일정 시간(seconds) 대기 후 sleep mode로 전환
            with open( assistant.status_path, "r") as f:
                    status = f.read()
                    if "AWAKE" in status: 
                        # print('sadfjhsfdahkfdsaj')
                        assistant.time = time.time()
                        assistant.chstatus("WAITING_FOR_USER_INPUT")
                    
            # print('elapsed time = ',time.time() - assistant.time)

            if time.time() - assistant.time > patience_time:
                print('--------- I am sleeping Zzz ---------')
                assistant.chstatus("SLEEP_MODE")
                status = "SLEEP_MODE"
                           
            if "SLEEP_MODE" in status:
                process_sleep_mode(assistant,responses)
            else :
                process_voice_input(assistant,responses)
            
def chatGPT(assistant = GPTAssistant(language="Kor")) :
    RETRY_DELAY = 1 #s
    assistant.start_time = time.time()
    try :
         _main(assistant)
         print("python thread end")
    except DeadlineExceeded: # 신호 처리 도중 timeout 발생
        h,min, s = elapsed_time(assistant.start_time)
        print(f"Elapsed Time : {h}h {min}min {s}s")
        print(f"Retry in {RETRY_DELAY} seconds(DeadlineExceed)", end= "\n\n")
        time.sleep(RETRY_DELAY) #s
        _main(assistant)

if __name__ == "__main__":
    RayAssistant = GPTAssistant(language="Kor", use_robot=False)
    chatGPT(RayAssistant)