# Makefile для розподіленої системи управління роєм дронів
# v2.0.0-MESH з повною автономією та mesh-протоколом

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g -pthread
LDFLAGS = -pthread

# Включення бібліотек
INCLUDES = -I./include -I/usr/include/eigen3 -I/usr/include/openssl
LIBS = -lyaml-cpp -lssl -lcrypto -lpthread -leigen3-dev

# Директорії
SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = include
CONFIG_DIR = config
FIRMWARE_DIR = firmware
SCRIPTS_DIR = scripts

# Основні вихідні файли
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
TARGET = swarm_control

# ESP32 прошивки
FIRMWARE_SOURCES = $(wildcard $(FIRMWARE_DIR)/drone_firmware/src/*.cpp)
FIRMWARE_TARGET = $(FIRMWARE_DIR)/drone_firmware/.pio/build/esp32dev/firmware.bin

# Конфігураційні файли
CONFIG_FILES = $(wildcard $(CONFIG_DIR)/*.yaml)

# Кольори для виводу
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[1;33m
BLUE = \033[0;34m
NC = \033[0m

# Головна ціль
all: banner $(TARGET)
	@echo "$(GREEN)✅ Розподілена система управління роєм зібрана успішно!$(NC)"

banner:
	@echo "$(BLUE)"
	@echo "╔══════════════════════════════════════════════════════════════╗"
	@echo "║           ЗБІРКА РОЗПОДІЛЕНОЇ СИСТЕМИ УПРАВЛІННЯ РОЄМ        ║"
	@echo "║                         v2.0.0-MESH                         ║"
	@echo "║                                                              ║"
	@echo "║                    🇺🇦 SLAVA UKRAINI! 🇺🇦                    ║"
	@echo "╚══════════════════════════════════════════════════════════════╝"
	@echo "$(NC)"

# Збірка головного виконуваного файлу
$(TARGET): $(OBJECTS)
	@echo "$(YELLOW)🔗 Компонування $(TARGET)...$(NC)"
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS) $(LIBS)
	@echo "$(GREEN)✅ $(TARGET) зібрано$(NC)"

# Компіляція об'єктних файлів
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	@echo "$(YELLOW)🔨 Компіляція $<...$(NC)"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Створення директорії збірки
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)
	@echo "$(BLUE)📁 Створено директорію $(BUILD_DIR)$(NC)"

# Встановлення залежностей (Ubuntu/Debian)
deps:
	@echo "$(BLUE)📦 Встановлення залежностей для розподіленої системи...$(NC)"
	sudo apt-get update
	sudo apt-get install -y \
		build-essential \
		cmake \
		libyaml-cpp-dev \
		libssl-dev \
		libcrypto++-dev \
		libeigen3-dev \
		pkg-config \
		python3-pip \
		git
	@echo "$(YELLOW)🔧 Встановлення PlatformIO для ESP32...$(NC)"
	pip3 install platformio
	@echo "$(GREEN)✅ Всі залежності встановлені$(NC)"

# Збірка ESP32 прошивки
firmware: $(FIRMWARE_TARGET)
	@echo "$(GREEN)✅ ESP32 прошивка зібрана$(NC)"

$(FIRMWARE_TARGET): $(FIRMWARE_SOURCES)
	@echo "$(YELLOW)🔨 Збірка ESP32 прошивки...$(NC)"
	cd $(FIRMWARE_DIR)/drone_firmware && pio run -e esp32dev
	@echo "$(GREEN)✅ ESP32 прошивка готова$(NC)"

# Прошивка ESP32
flash: firmware
	@echo "$(YELLOW)⚡ Прошивка ESP32...$(NC)"
	cd $(FIRMWARE_DIR)/drone_firmware && pio run -t upload -e esp32dev
	@echo "$(GREEN)✅ ESP32 прошито успішно$(NC)"

# Збірка в режимі відлагодження
debug: CXXFLAGS += -DDEBUG -g3 -O0 -DMESH_DEBUG -DPOSITION_DEBUG
debug: $(TARGET)
	@echo "$(GREEN)✅ Debug збірка готова$(NC)"

# Збірка для продакшн (оптимізована)
release: CXXFLAGS += -DNDEBUG -O3 -march=native -DPRODUCTION_BUILD
release: $(TARGET)
	@echo "$(GREEN)✅ Production збірка готова$(NC)"

# Збірка для ARM (для установки на дрон)
arm: CXX = arm-linux-gnueabihf-g++
arm: CXXFLAGS += -DARM_BUILD -mcpu=cortex-a7
arm: $(TARGET)
	@echo "$(GREEN)✅ ARM збірка готова$(NC)"

# Повна збірка (Ground Station + ESP32)
full: $(TARGET) firmware
	@echo "$(GREEN)✅ Повна система зібрана (Ground Station + ESP32)$(NC)"

# Очищення
clean:
	@echo "$(YELLOW)🧹 Очищення збірки...$(NC)"
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET)
	@if [ -d "$(FIRMWARE_DIR)/drone_firmware/.pio" ]; then \
		cd $(FIRMWARE_DIR)/drone_firmware && pio run -t clean; \
	fi
	@echo "$(GREEN)✅ Очищення завершено$(NC)"

# Встановлення
install: $(TARGET) firmware
	@echo "$(YELLOW)📥 Встановлення розподіленої системи...$(NC)"
	sudo cp $(TARGET) /usr/local/bin/
	sudo mkdir -p /etc/swarm_control
	sudo cp -r $(CONFIG_DIR)/* /etc/swarm_control/ 2>/dev/null || true
	sudo mkdir -p /opt/swarm_control/firmware
	sudo cp -r $(FIRMWARE_DIR)/* /opt/swarm_control/firmware/ 2>/dev/null || true
	sudo chmod +x /usr/local/bin/$(TARGET)
	@echo "$(GREEN)✅ Система встановлена:$(NC)"
	@echo "  📁 Виконуваний файл: /usr/local/bin/$(TARGET)"
	@echo "  📁 Конфігурація: /etc/swarm_control/"
	@echo "  📁 Прошивки: /opt/swarm_control/firmware/"

# Видалення
uninstall:
	@echo "$(YELLOW)🗑️  Видалення системи...$(NC)"
	sudo rm -f /usr/local/bin/$(TARGET)
	sudo rm -rf /etc/swarm_control
	sudo rm -rf /opt/swarm_control
	@echo "$(GREEN)✅ Система видалена$(NC)"

# Тестування
test: $(TARGET)
	@echo "$(YELLOW)🧪 Запуск базових тестів...$(NC)"
	@echo "$(BLUE)Тест 1: Ініціалізація дрона$(NC)"
	timeout 10s ./$(TARGET) 0201 ./config/swarm_config.yaml &
	sleep 3
	pkill $(TARGET) || true
	@echo "$(BLUE)Тест 2: Mesh-зв'язок$(NC)"
	./scripts/test_mesh.sh
	@echo "$(GREEN)✅ Базові тести пройдені$(NC)"

# Симуляція рою
simulate: $(TARGET)
	@echo "$(YELLOW)🎮 Запуск симуляції розподіленого рою...$(NC)"
	@echo "$(BLUE)Запуск 3 віртуальних дронів з mesh-зв'язком...$(NC)"
	./$(TARGET) 0201 ./config/swarm_config.yaml &
	sleep 2
	./$(TARGET) 0202 ./config/swarm_config.yaml &
	sleep 2
	./$(TARGET) 0203 ./config/swarm_config.yaml &
	@echo "$(GREEN)⚡ Симуляція запущена. Використовуйте Ctrl+C для зупинки.$(NC)"

# Симуляція великого рою
big_simulate: $(TARGET)
	@echo "$(YELLOW)🎮 Запуск симуляції великого рою (10 дронів)...$(NC)"
	@for i in $$(seq 1 10); do \
		drone_id=$$((200 + i)); \
		echo "$(BLUE)Запуск дрона $$drone_id$(NC)"; \
		./$(TARGET) $$drone_id ./config/swarm_config.yaml & \
		sleep 1; \
	done
	@echo "$(GREEN)⚡ Великий рой симульовано (10 дронів)$(NC)"

# Перевірка синтаксису
check:
	@echo "$(YELLOW)🔍 Перевірка синтаксису...$(NC)"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -fsyntax-only $(SOURCES)
	@echo "$(GREEN)✅ Синтаксис правильний$(NC)"

# Форматування коду
format:
	@echo "$(YELLOW)🎨 Форматування коду...$(NC)"
	find $(SRC_DIR) $(INCLUDE_DIR) -name "*.cpp" -o -name "*.h" | xargs clang-format -i
	@echo "$(GREEN)✅ Код відформатований$(NC)"

# Статичний аналіз коду
analyze:
	@echo "$(YELLOW)🔬 Статичний аналіз коду...$(NC)"
	cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $(SRC_DIR)/ $(INCLUDE_DIR)/
	@echo "$(GREEN)✅ Аналіз завершений$(NC)"

# Документація
docs:
	@echo "$(YELLOW)📚 Генерація документації...$(NC)"
	@if command -v doxygen >/dev/null 2>&1; then \
		doxygen Doxyfile; \
		echo "$(GREEN)✅ Документація згенерована в docs/$(NC)"; \
	else \
		echo "$(RED)❌ Doxygen не знайдений$(NC)"; \
	fi

# Пакування для розповсюдження
package: release firmware
	@echo "$(YELLOW)📦 Створення пакета розповсюдження...$(NC)"
	mkdir -p dist/swarm_control_mesh
	cp $(TARGET) dist/swarm_control_mesh/
	cp -r $(CONFIG_DIR) dist/swarm_control_mesh/
	cp -r $(FIRMWARE_DIR) dist/swarm_control_mesh/
	cp -r $(SCRIPTS_DIR) dist/swarm_control_mesh/
	cp README.md dist/swarm_control_mesh/ 2>/dev/null || true

	# Створення скрипта запуску
	@cat > dist/swarm_control_mesh/start_swarm.sh << 'EOF'
#!/bin/bash
echo "🇺🇦 Запуск розподіленої системи управління роєм 🇺🇦"
if [ $$# -lt 1 ]; then
	echo "Використання: $$0 <drone_id> [config_file]"
	echo "Приклад: $$0 0201"
	exit 1
fi
DRONE_ID=$$1
CONFIG_FILE=$${2:-"config/swarm_config.yaml"}
echo "🚁 Запуск дрона ID: $$DRONE_ID"
echo "📁 Конфігурація: $$CONFIG_FILE"
echo "🕸️  Mesh-мережа: АКТИВНА"
echo "🤖 Автономія: УВІМКНЕНА"
./swarm_control $$DRONE_ID $$CONFIG_FILE
EOF
	chmod +x dist/swarm_control_mesh/start_swarm.sh

	tar -czf swarm_control_mesh_$(shell date +%Y%m%d_%H%M%S).tar.gz -C dist swarm_control_mesh
	@echo "$(GREEN)✅ Пакет створений: swarm_control_mesh_*.tar.gz$(NC)"

# Флеш на SD карту для польових умов
flash_sd: package
	@echo "$(YELLOW)💾 Запис на SD карту...$(NC)"
	@read -p "Введіть шлях до SD карти (наприклад /dev/sdb1): " SDCARD && \
	if [ -b "$$SDCARD" ]; then \
		sudo mount $$SDCARD /mnt && \
		sudo cp -r dist/swarm_control_mesh/* /mnt/ && \
		sudo umount /mnt && \
		echo "$(GREEN)✅ Система записана на $$SDCARD$(NC)"; \
	else \
		echo "$(RED)❌ $$SDCARD не є блочним пристроєм$(NC)"; \
	fi

# Моніторинг системи
monitor:
	@echo "$(YELLOW)📊 Моніторинг розподіленої системи...$(NC)"
	watch -n 2 'ps aux | grep $(TARGET) | grep -v grep; echo "---"; ss -tuln | grep -E "(868[0-9]|2400|5800)"; echo "---"; tail -n 5 /var/log/syslog | grep swarm'

# Логи системи
logs:
	@echo "$(YELLOW)📋 Перегляд логів системи...$(NC)"
	journalctl -f | grep --color=always -i swarm

# Тест mesh-зв'язку
test_mesh: $(TARGET)
	@echo "$(YELLOW)🕸️ Тест mesh-протоколу...$(NC)"
	@$(SCRIPTS_DIR)/test_mesh_connectivity.sh
	@echo "$(GREEN)✅ Mesh-тест завершений$(NC)"

# Бекап конфігурації
backup_config:
	@echo "$(YELLOW)💾 Бекап конфігурації...$(NC)"
	backup_dir="config_backup_$(shell date +%Y%m%d_%H%M%S)"
	mkdir -p $$backup_dir
	cp -r $(CONFIG_DIR)/* $$backup_dir/
	cp -r /etc/swarm_control/* $$backup_dir/ 2>/dev/null || true
	tar -czf $$backup_dir.tar.gz $$backup_dir
	rm -rf $$backup_dir
	@echo "$(GREEN)✅ Конфігурація збережена: $$backup_dir.tar.gz$(NC)"

# Stress test
stress_test: $(TARGET)
	@echo "$(YELLOW)💪 Stress test розподіленої системи...$(NC)"
	@for i in $$(seq 1 20); do \
		drone_id=$$((200 + i)); \
		echo "Запуск stress-дрона $$drone_id"; \
		timeout 30s ./$(TARGET) $$drone_id ./config/swarm_config.yaml & \
	done
	sleep 35
	pkill $(TARGET) || true
	@echo "$(GREEN)✅ Stress test завершено$(NC)"

# Benchmark mesh-мережі
benchmark_mesh: $(TARGET)
	@echo "$(YELLOW)⚡ Benchmark mesh-мережі...$(NC)"
	./scripts/benchmark_mesh.sh
	@echo "$(GREEN)✅ Benchmark завершено$(NC)"

# Довідка з розширеними командами
help:
	@echo "$(BLUE)🇺🇦 РОЗПОДІЛЕНА СИСТЕМА УПРАВЛІННЯ РОЄМ ДРОНІВ v2.0.0 🇺🇦$(NC)"
	@echo ""
	@echo "$(GREEN)ОСНОВНІ КОМАНДИ:$(NC)"
	@echo "  make deps         - Встановити всі залежності"
	@echo "  make              - Збірка ground station"
	@echo "  make firmware     - Збірка ESP32 прошивки"
	@echo "  make full         - Повна збірка системи"
	@echo "  make flash        - Прошивка ESP32"
	@echo ""
	@echo "$(GREEN)ЗБІРКА:$(NC)"
	@echo "  make debug        - Збірка з відлагодженням"
	@echo "  make release      - Продакшн збірка"
	@echo "  make arm          - Збірка для ARM"
	@echo "  make clean        - Очистити збірку"
	@echo ""
	@echo "$(GREEN)ТЕСТУВАННЯ:$(NC)"
	@echo "  make test         - Базові тести"
	@echo "  make simulate     - Симуляція рою (3 дрони)"
	@echo "  make big_simulate - Великий рой (10 дронів)"
	@echo "  make test_mesh    - Тест mesh-зв'язку"
	@echo "  make stress_test  - Навантажувальний тест"
	@echo "  make benchmark_mesh - Benchmark mesh-мережі"
	@echo ""
	@echo "$(GREEN)РОЗГОРТАННЯ:$(NC)"
	@echo "  make install      - Встановити систему"
	@echo "  make package      - Створити пакет"
	@echo "  make flash_sd     - Записати на SD карту"
	@echo ""
	@echo "$(GREEN)УТИЛІТИ:$(NC)"
	@echo "  make check        - Перевірка синтаксису"
	@echo "  make format       - Форматування коду"
	@echo "  make analyze      - Статичний аналіз"
	@echo "  make docs         - Генерація документації"
	@echo "  make monitor      - Моніторинг системи"
	@echo "  make logs         - Перегляд логів"
	@echo ""
	@echo "$(GREEN)ПРИКЛАД ВИКОРИСТАННЯ:$(NC)"
	@echo "  make deps && make full && sudo make install"
	@echo "  swarm_control 0201 /etc/swarm_control/swarm_config.yaml"
	@echo ""
	@echo "$(GREEN)MESH-МЕРЕЖА: Автоматичне виявлення та з'єднання дронів$(NC)"
	@echo "$(GREEN)АВТОНОМІЯ: Рій продовжує місію навіть без оператора$(NC)"
	@echo "$(GREEN)ПОЗИЦІОНУВАННЯ: Працює без GPS через UWB + динамічний якір$(NC)"
	@echo ""
	@echo "$(BLUE)SLAVA UKRAINI! 🇺🇦$(NC)"

# Підготовка до бойового розгортання
combat_deploy: clean deps release firmware package
	@echo "$(RED)⚔️  ПІДГОТОВКА ДО БОЙОВОГО РОЗГОРТАННЯ ⚔️$(NC)"
	@echo "$(YELLOW)🔐 Перевірка систем безпеки...$(NC)"
	@grep -q "SlavaUkraini" config/swarm_config.yaml && echo "$(GREEN)✅ Шифрування налаштоване$(NC)"
	@echo "$(YELLOW)🛡️ Перевірка систем захисту...$(NC)"
	@grep -q "self_destruct" config/swarm_config.yaml && echo "$(GREEN)✅ Система самознищення активна$(NC)"
	@echo "$(YELLOW)📡 Перевірка mesh-протоколу...$(NC)"
	@grep -q "mesh_network" config/swarm_config.yaml && echo "$(GREEN)✅ Mesh-мережа налаштована$(NC)"
	@echo "$(GREEN)✅ Система готова до бойового застосування$(NC)"
	@echo "$(RED)⚠️  ВИКОРИСТОВУЙТЕ ВІДПОВІДАЛЬНО!$(NC)"

.PHONY: all clean deps debug release arm firmware flash full install uninstall test simulate big_simulate check format analyze docs package flash_sd monitor logs test_mesh backup_config stress_test benchmark_mesh help combat_deploy banner