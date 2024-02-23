# config.py

# Google Cloud API
GOOGLE_APPLICATION_CREDENTIALS = 'beaming-ion-393306-4002f5e2f913.json'#임세혁 박사님 계정

# OpenAI API
OPENAI_API_KEY = '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'  #임세혁 박사님 계정

# OpenAI API
CURRENT_PATH = 'D:/OpenFace - mouth/x64/Release/AIspeaker_JA/'

data_sync_file_path = CURRENT_PATH + "robot_status.txt"


def update_data_sync(status):
    with open(data_sync_file_path, "w") as f:
        f.write(status)
        # print(f"write {status}")

from time import sleep
def main():
    sleep(100000)
    print("정상 작동.")