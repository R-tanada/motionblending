import sys
import os
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout, QRadioButton, QButtonGroup, QMessageBox
import csv

class SurveyApp(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        self.questions = [
            "ロボットのアシストを信頼できた。",
            "安心して作業を行うことができた。",
            "ロボットの動きが不安に感じた。",
        ]

        self.scores = []

        self.name_input = QLineEdit(self)
        self.condition_input = QLineEdit(self)

        self.layout = QVBoxLayout()
        self.layout.addWidget(QLabel("回答者の名前："))
        self.layout.addWidget(self.name_input)
        self.layout.addWidget(QLabel("条件の名前："))
        self.layout.addWidget(self.condition_input)

        annotation_layout = QHBoxLayout()
        annotation_layout.addWidget(QLabel("1: 全くそう感じない"))
        annotation_layout.addStretch()  # Stretch to center-align the annotations
        annotation_layout.addWidget(QLabel("7: 非常にそう感じる"))

        self.layout.addLayout(annotation_layout)

        for idx, question in enumerate(self.questions):
            label = QLabel(question)

            radio_layout = QHBoxLayout()
            button_group = QButtonGroup(self)

            for i in range(1, 8):
                radio_button = QRadioButton(str(i))
                button_group.addButton(radio_button, i)
                radio_layout.addWidget(radio_button)

            self.layout.addWidget(label)
            self.layout.addLayout(radio_layout)

            self.scores.append(button_group)

        submit_button = QPushButton("回答を保存", self)
        submit_button.clicked.connect(self.save_answers)
        self.layout.addWidget(submit_button)

        self.setLayout(self.layout)

        self.setGeometry(300, 300, 600, 400)
        self.setWindowTitle('リッカートスケールアンケート')

    def save_answers(self):
        name = self.name_input.text()
        condition = self.condition_input.text()

        # フォルダが存在しない場合は作成
        folder_path = os.path.join(name, "")
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        filename = os.path.join(folder_path, f"{condition}.csv")

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Question", "Score"])
            for idx, button_group in enumerate(self.scores):
                question = self.questions[idx]
                score = button_group.checkedId()
                writer.writerow([question, score])

        msg_box = QMessageBox()
        msg_box.setWindowTitle("保存完了")
        msg_box.setText(f"回答が {filename} に保存されました。")
        msg_box.exec_()

        sys.exit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    survey_app = SurveyApp()
    survey_app.show()
    sys.exit(app.exec_())
