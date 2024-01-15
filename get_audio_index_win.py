import pyaudio

iaudio = pyaudio.PyAudio()
for i in range(0, iaudio.get_device_count()):
    index = iaudio.get_device_info_by_index(i)
    if (
        index["maxInputChannels"] == 0
        and "Sound Blaster Play! 3" in index["name"]
        and index["defaultSampleRate"] == 44100
    ):
        print(index)
