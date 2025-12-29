# Laba4
Задание: нарисовать первую букву своего имени "И" с помощью Ros2 в TurtleSim (черепаха).

Что нужно сделать, чтобы запустить:
- Создать workspace (если ещё нет): `source /opt/ros/iron/setup.bash && mkdir -p ~/ros2_ws/src`
- Склонировать репозиторий с заданием: `cd ~/ros2_ws/src && git clone https://github.com/AlexGT555/lesson_04.git`
- Добавить файл ноды `draw_ilyas.py` в `~/ros2_ws/src/lesson_04/lesson_04/` и прописать запуск в `setup.py` в `console_scripts`: `'draw_ilyas = lesson_04.draw_ilyas:main',`
- Собрать пакет и подключить окружение: `cd ~/ros2_ws && colcon build --packages-select lesson_04 && source ~/ros2_ws/install/local_setup.bash`
- Запустить TurtleSim: `source /opt/ros/iron/setup.bash && ros2 run turtlesim turtlesim_node`
- Запустить ноду для рисования буквы: `source /opt/ros/iron/setup.bash && source ~/ros2_ws/install/local_setup.bash && ros2 run lesson_04 draw_ilyas`
