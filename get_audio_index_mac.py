import pyaudio

iaudio = pyaudio.PyAudio()
for i in range(0, iaudio.get_device_count()):
    index = iaudio.get_device_info_by_index(i)
    print(index)
    # if (
    #     index["maxInputChannels"] == 0
    #     and index["name"] == "スピーカー (Sound Blaster Play! 3)"
    # ):
    #     print(index)