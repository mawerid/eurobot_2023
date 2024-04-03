# Подключение к RPi4 по SSH
```Перед тем как начать подключение убедиться, что компьютер и RPi 4 подключены к одной точке доступа```


	Название точки достпуа: engi

	Пароль: neutrhino

```Далее, можно узнать IP адрес и подключиться через приложение "REMINNA" для Ubuntu, через удаленный рабочий стол для Windows```

# Для запуска микро_роса
```console
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
# Просмотр подключенных топиков
```console
ros2 topic list
```
# При подключенной пико, там можно увидеть такую картину

	/rosout
	/pico_publisher
	/ещё какой-то базовый

# Для просмотра того, что есть в топике

```console
ros2 topic echo <name_topic>
```
# Включение камеры
## Первый способ

```console
ros2 run tank ArUco
```
## Второй способ

```console
ros2 run v4l2_camera v4l2_camera_node
```
```В другом терминале нужно написать:```

```console
ros2 run rqt_image_view rqt_image_view
```

# Включение управления моторами

## Электромоторы
```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Шаговые моторы
```console
ros2 run tank minimal_publisher
```

```Далее должен появиться топик ``` __/stepper_topic__ ```Для того чтобы передать информацию на моторы в``` __другом__ ``` терминале нужно написать```

```console
ros2 topic pub --once /stepper_topic geometry_msgs/msg/Vector3 "{x: 1, y: 2, z: 3}"
```
## Сервомоторы
```Предварительно запустить в отдельном терминале```
```console
ros2 run tank minimal_publisher
```

```Далее должен появиться топик ``` __/servo_topic__ ```Для того чтобы передать информацию на сервы в``` __другом__ ``` терминале нужно написать```

```console
ros2 topic pub --once /servo_topic geometry_msgs/msg/Vector3 "{x: 1, y: 2, z: 0}"
```