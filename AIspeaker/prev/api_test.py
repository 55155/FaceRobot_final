import io
import os
import wave
import pyaudio
from google.cloud import speech_v1p1beta1 as speech
from google.cloud.speech_v1p1beta1 import enums

def transcribe_streaming():
    client = speech.SpeechClient()
    config = speech.types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="en-US",
        audio_channel_count=2,
        enable_separate_recognition_per_channel=True,
    )
    streaming_config = speech.types.StreamingRecognitionConfig(
        config=config,
        interim_results=True
    )
    mic = pyaudio.PyAudio()
    stream = mic.open(rate=16000, channels=2, format=pyaudio.paInt16, input=True, frames_per_buffer=1024)
    audio_generator = stream.read(1024)

    requests = (
        speech.types.StreamingRecognizeRequest(audio_content=content)
        for content in audio_generator
    )

    responses = client.streaming_recognize(streaming_config, requests)
    for response in responses:
        for result in response.results:
            print("Transcript: {}".format(result.alternatives[0].transcript))