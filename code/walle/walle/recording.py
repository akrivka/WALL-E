import cv2
import time
import sounddevice as sd
from scipy.io.wavfile import write
import wavio as wv
import threading
import time

CHUNK_SECONDS = 10
chunk = 0
while True:
    # start webcam
    cap = cv2.VideoCapture(0)  # 0 is usually the default webcam

    # video recording
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .avi file format
    out = cv2.VideoWriter(f'chunks/chunk{chunk}.mp4', fourcc, 10.0, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
        
    # Audio recording
    freq = 44100
    recording = sd.rec(int(CHUNK_SECONDS * freq), samplerate=freq, channels=2)
    start = time.time()
    iteration = 0
    try:
        while time.time() - start < CHUNK_SECONDS:
            print('Time:', round(time.time() - start, 2), 'Rate:', iteration / (max(time.time() - start, 0.01)), end='\r')
            iteration += 1
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            out.write(frame)
    finally:
        write(f"chunks/chunk{chunk}.wav", freq, recording)
        # wv.write("recording1.wav", recording, freq, sampwidth=2)
        cap.release()
        out.release()
        chunk += 1