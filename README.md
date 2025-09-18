# **Проект "TO-DO"**

## **Установка и настройка**

### **Необходимые компоненты**
- **Docker** ([Инструкция по установке](https://docs.docker.com/engine/install/))

### **Быстрый старт с Docker**

```bash
git clone git@github.com:Robosoft-MIK-2025/maneuver.git
cd maneuver
docker compose up --build
```

---

## **Руководство по разработке**

### **1. Локальная разработка (без Docker)**
Установите необходимые пакеты Gazebo:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
# Очистка возможных конфликтующих версий Gazebo
sudo apt remove 'gazebo*' 'libgazebo*' 'gz-tools*'
sudo apt autoremove
```

### **2. Разработка в Docker**
Скачайте репозиторий и перейдите в его корень
```bash
git clone git@github.com:Robosoft-MIK-2025/maneuver.git
cd maneuver
```

Запустите контейнер:
```bash
docker compose up --build terminal
```

Подключитесь к контейнеру из других терминалов:
```bash
docker compose exec terminal bash
```

### **3. Сборка Docker**

Сборка образа:
```bash
docker build -t image_name -f Dockerfile .
```

Тег образа:
```bash
docker tag image_name fabook/mik:v0.0.5
```

Загрузка образа в Docker Hub:
```bash
docker push fabook/mik:v0.0.5
```

Выгрузка образа из Docker Hub:
```bash
docker pull fabook/mik:v0.0.5
```

---

## **Инструкция по сборке пакета и работе с репозиторием**
Чтобы корректно собрать пакет с помощью colcon, выполните следующие шаги:

**Активация ROS-окружения**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```
Первая команда подключает системные пакеты ROS Humble, а вторая - ваши локальные пакеты из текущего рабочего пространства.

**Сборка пакета**
```bash
colcon build
```

**Работа с репозиторием GitHub**
```bash
git pull origin <имя_ветки>
```

---

## **Инструкция по запуску симуляции с дроном**

```bash
ros2 launch maneuver_bringup full_launch.py
```
Эта команда автоматически:

- Запускает QGroundControl (QGC) для управления дроном,
- Инициализирует симуляцию в Gazebo с моделью дрона, оснащённого стереокамерой,
- Поднимает MicroXRCE-агент для связи,
- Пробрасывает топик облака точек с камеры: /depth_camera/points (тип сообщения: sensor_msgs/msg/PointCloud2).

---

## **Запуск статической карты для тестирования MoveIt**
Поскольку сейчас SLAM не готов, то был создан publisher, который публикует статическую карту из файла в топик /octomap.
Запускается командой
```bash
ros2 run maneuver_path_planner octomap_publisher
```
---

## **Установка Docker-образа (альтернативная инструкция)**

**ПРЕДУПРЕЖДЕНИЕ: Это инструкция в разработке. Используйте на свой страх и риск!**

### **1. Установка образа**

Затем выполните:
```
git clone -b EgorBranch https://github.com/Robosoft-MIK-2025/maneuver.git
cd maneuver
docker compose up terminal
```

Объяснение кода:
- `git clone -b EgorBranch https://github.com/Robosoft-MIK-2025/maneuver.git` загружает последние файлы из репозитория MIK github из ветки `EgorBranch`
- `cd maneuver` переходит в папку `maneuver`
- `docker compose up terminal` запускает контейнер (также загружает все зависимости при первом запуске)

### **2. Запуск PX4 симуляции**
1. Убедитесь, что docker-контейнер работает (открыт терминал с запущенной командой `docker compose up terminal`)
2. Откройте еще два терминала и подключите их к docker-контейнеру с помощью `docker compose exec terminal bash`
3. В одном терминале запустите агент с настройками для подключения к клиенту uXRCE-DDS, работающему в симуляторе: `MicroXRCEAgent udp4 -p 8888`
4. В другом терминале запустите симулятор:  `cd /home/mobile/PX4-Autopilot && make px4_sitl gz_x500`

---

## **Правила работы с Git и коммитами**

### **Основные теги (типы коммитов)**
| Тег         | Описание                                                                 |
|-------------|--------------------------------------------------------------------------|
| feat        | Новая функциия. Пример: `feat: добавлена аутентификация`        |
| fix         | Исправление ошибки. Пример: `fix: исправлен краш при null-вводе`        |
| docs        | Изменения в документации. Пример: `docs: обновлен README.md`            |
| style       | Изменения форматирования (пробелы, запятые). Пример: `style: форматирование по PEP8` |
| refactor    | Рефакторинг кода (без изменения функционала). Пример: `refactor: оптимизация функции X` |
| perf        | Улучшение производительности. Пример: `perf: уменьшено время загрузки` |
| test        | Изменения, связанные с тестами. Пример: `test: добавлено покрытие API` |
| chore       | Технические задачи (зависимости, конфиги). Пример: `chore: обновление webpack` |
| ci          | Изменения CI/CD (GitHub Actions, GitLab CI). Пример: `ci: добавлен деплой на staging` |
| build       | Изменения в системе сборки. Пример: `build: добавлен Dockerfile`       |
| revert      | Отмена предыдущего коммита. Пример: `revert: отмена коммита 123abc`    |

### **Дополнительные правила**
1. **Сообщение** должно быть четким и лаконичным.
   - Неправильно: `fix: баг`
   - Правильно: `fix: исправлена ошибка отправки формы`

2. **Тело коммита** (опционально) - подробное описание изменений.
   ```
   fix: устранена утечка памяти в модуле X

   Утечка возникала из-за незакрытых соединений с БД при долгих сессиях.
   Добавлен `cleanup()` для корректного освобождения ресурсов.
   ```

3. **Футер** (опционально) - ссылки на задачи, критические изменения.
   ```
   feat: добавлена поддержка WebSocket

   BREAKING CHANGE: Устаревший API `/chat` больше не поддерживается.
   Closes #123
   ```

---

## **Как загрузить изменения в отдельную ветку**

```bash
# 1. Создать новую ветку
git checkout -b <имя_ветки>
```

# 2. Проверить текущую ветку
```bash
git branch
```

# 3. Добавить изменения
```bash
git add .
```

# 4. Создать коммит
```bash
git commit -m "<тип>: <описание>"
```

# 5. Загрузить ветку на GitHub
```bash
git push -u origin <имя_ветки>
```
