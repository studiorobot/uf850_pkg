
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFontDatabase
import os
from setup_steps import SetupWindow
import sys
from PyQt5.QtGui import QIcon


if __name__ == "__main__":
    app = QApplication(sys.argv)
    # Load Norwester font
    font_path = os.path.join("resources", "fonts", "norwester.otf")
    font_id = QFontDatabase.addApplicationFont(font_path)
    families = QFontDatabase.applicationFontFamilies(font_id)
    if families:
        app.setStyleSheet(f"* {{ font-family: '{families[0]}'; }}")
    else:
        print("Failed to load Norwester font.")
    window = SetupWindow()
    window.show()
    window.setWindowIcon(QIcon("resources/logo.png"))
    app.setStyleSheet(app.styleSheet() + """
        QWidget {
            background-color: white;
        }
    """)

    sys.exit(app.exec_())
