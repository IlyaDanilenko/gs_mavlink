# Описание пакета gs_mavlink

## Описание:
Данный пакет необходим для связи GCS по протаколу MAVLink

## Состав пакета:
1. Ноды:
    * mavlink_node
2. Файлы запуска:
    * mavlink_pioneer.launch - заупск системы вместе с MAVLink сервером

## Описание нод:
### 1. mavlink_node
Нода MAVLink UDP сервера

## Необходимые пакеты:
1. Python:
    * pymavlink
2. ROS:
    * gs_core
    * gs_board
    * gs_flight
    * gs_sensors
    * gs_navigation

## Примечание:
* Для работы всех нод в данном пакете необходима запущенная нода ros_plaz_node.py из пакета gs_core.
* Автопилот версии ниже 1.5.7 не поддерживается.
