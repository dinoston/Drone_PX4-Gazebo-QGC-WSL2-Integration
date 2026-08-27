"""Qt widget for RGB, depth and segmentation streams."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QWidget


class CameraPanel(QFrame):
    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        layout = QGridLayout(self)
        title_label = QLabel(title)
        title_label.setObjectName("sensorTitle")
        self.image_label = QLabel("영상 대기 중")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(280, 180)
        self.image_label.setStyleSheet("background:#10151d; color:#7f8b9a;")
        layout.addWidget(title_label, 0, 0)
        layout.addWidget(self.image_label, 1, 0)
        self._image = QImage()

    def set_encoded_image(self, data: bytes) -> None:
        image = QImage()
        if not image.loadFromData(data):
            self.image_label.setText("영상 디코딩 실패")
            return
        self._image = image
        self._refresh_pixmap()

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._image.isNull():
            return
        pixmap = QPixmap.fromImage(self._image).scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(pixmap)


class CameraViewer(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        self.panels = {
            "RGB": CameraPanel("RGB"),
            "Depth": CameraPanel("Depth"),
            "Segmentation": CameraPanel("Segmentation"),
        }
        layout.addWidget(self.panels["RGB"], 0, 0)
        layout.addWidget(self.panels["Depth"], 0, 1)
        layout.addWidget(self.panels["Segmentation"], 1, 0, 1, 2)

    def update_images(self, images: dict[str, bytes]) -> None:
        for name, data in images.items():
            panel = self.panels.get(name)
            if panel is not None:
                panel.set_encoded_image(data)
