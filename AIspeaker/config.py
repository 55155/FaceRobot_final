# config.py

# Google Cloud API
GOOGLE_APPLICATION_CREDENTIALS = 'beaming-ion-393306-4002f5e2f913.json'#임세혁 박사님 계정

# OpenAI API
# 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
# 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 # 2023.11.01 갱신
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
    print("정상 작동.")