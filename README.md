# Laba4
Задание: нарисовать первую букву своего имени "И" с помощью Ros2 в TurtleSim (черепаха).

Что нужно сделать, чтобы запустить:
- Создать workspace `~/ros2_ws` (если ещё нет) и папку `src` внутри него.
- Склонировать репозиторий с заданием в `~/ros2_ws/src` и добавить/создать файл с нодой для рисования буквы (например `lesson_04/lesson_04/draw_ilyas.py`).
- Прописать запуск ноды в `setup.py` через `console_scripts`, затем собрать пакет командой `colcon build --packages-select lesson_04` и выполнить `source /opt/ros/iron/setup.bash` и `source ~/ros2_ws/install/local_setup.bash`.
- В одном терминале запустить TurtleSim командой `ros2 run turtlesim turtlesim_node`, а во втором — запустить свою ноду командой `ros2 run lesson_04 draw_ilyas`.
