import random
import time
import PySimpleGUI as sg
import threading

target_right = [1, 2, 3, 4]
target_left = [1, 2, 3, 4]

target_index_right = []
target_index_left = []

while True:
    target_index_left = random.sample(range(1, 5), k = 2)
    target_index_right = random.sample(range(1, 5), k = 2)

    if (target_index_right[0] != target_index_left[0]) and (target_index_right[1] != target_index_left[1]):
        break

    time.sleep(0.5)

target = target_right[target_index_right[0]]

def window():
    layout = [    [sg.Text('下の入力欄になにか入力して「GO」ボタンを押してください。:')],
                        [sg.Text('「Clear」ボタンを押すと、画面をクリアします。:')],
                        [sg.Text('入力欄'),sg.In(key='-IN-')],
                        [sg.Output(size=(50,10), key='-OUTPUT-')],
                        [sg.Button('Go'), sg.Button('Clear'), sg.Button('Exit')]  ]

    window = sg.Window('　テキスト表示アプリ　', layout)

    while True:

        #　ユーザからの入力を待ちます。入力があると、次の処理に進みます。
        event, values = window.read()

        #　テキスト入力欄に入力されたときのeventとvaluesを表示
        print('event＝',event, '、values＝',values)
        #　テキスト入力値のみを表示
        print('入力値＝',values['-IN-'])

        #　ウィンドウの右上の×を押したときの処理、「Exit」ボタンを押したときの処理
        if event in (sg.WIN_CLOSED, 'Exit'):
            break
        #　「Clear」ボタンを押したときの処理
        if event == 'Clear':
            #　「-OUTPUT-」領域を、空白で更新します。
            window['-OUTPUT-'].update('')

    window.close()


window_thread = threading.Thread(target=window)
window_thread.setDaemon(True)
window_thread.start()

try:
    while True:
        print('hello')

        time.sleep(0.5)

except KeyboardInterrupt:
    pass