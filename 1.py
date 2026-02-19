import cv2
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def extract_color_wheel_colors(image_path):
    """
    Извлекает средние цвета сегментов цветового круга
    """
    # Загрузка изображения
    img = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Преобразование в grayscale и blur
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, 5)
    
    # Обнаружение кругов
    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1, 
        minDist=50,
        param1=50, 
        param2=50, 
        minRadius=100, 
        maxRadius=500
    )
    
    if circles is None:
        print("Круг не найден! Попробуем альтернативный метод...")
        # Альтернативный метод - находим центр изображения
        height, width = img.shape[:2]
        center = (width // 2, height // 2)
        radius = min(height, width) // 2 - 10
    else:
        circles = np.round(circles[0, :]).astype("int")
        x, y, r = circles[0]
        center = (x, y)
        radius = r
    
    print(f"Центр: {center}, Радиус: {radius}")
    
    # Параметры для сегментов
    inner_radius = int(radius * 0.4)  # Внутренний радиус (где начинается цветное кольцо)
    outer_radius = int(radius * 0.95)  # Внешний радиус
    
    # Создаем маску круга
    mask = np.zeros(gray.shape, np.uint8)
    cv2.circle(mask, center, outer_radius, 255, -1)
    cv2.circle(mask, center, inner_radius, 0, -1)
    
    # Количество сегментов (определим автоматически или зададим)
    num_segments = 36  # Примерно по 10 градусов на сегмент
    
    segment_colors = []
    segment_angles = []
    
    for i in range(num_segments):
        angle_start = i * (360 / num_segments)
        angle_end = (i + 1) * (360 / num_segments)
        
        # Создаем маску для текущего сегмента
        segment_mask = np.zeros_like(mask)
        
        # Рисуем сектор
        point1 = (
            int(center[0] + inner_radius * np.cos(np.radians(angle_start))),
            int(center[1] - inner_radius * np.sin(np.radians(angle_start)))
        )
        point2 = (
            int(center[0] + outer_radius * np.cos(np.radians(angle_start))),
            int(center[1] - outer_radius * np.sin(np.radians(angle_start)))
        )
        point3 = (
            int(center[0] + outer_radius * np.cos(np.radians(angle_end))),
            int(center[1] - outer_radius * np.sin(np.radians(angle_end)))
        )
        point4 = (
            int(center[0] + inner_radius * np.cos(np.radians(angle_end))),
            int(center[1] - inner_radius * np.sin(np.radians(angle_end)))
        )
        
        pts = np.array([point1, point2, point3, point4], np.int32)
        cv2.fillPoly(segment_mask, [pts], 255)
        
        # Применяем маску к изображению
        masked_img = cv2.bitwise_and(img, img, mask=segment_mask)
        
        # Вычисляем средний цвет сегмента
        mean_val = cv2.mean(img, mask=segment_mask)[:3]
        b, g, r = mean_val
        
        # Проверяем, есть ли достаточно пикселей
        num_pixels = cv2.countNonZero(segment_mask)
        if num_pixels > 100:  # Минимальное количество пикселей
            segment_colors.append((r, g, b))  # RGB формат
            segment_angles.append(angle_start)
    
    # Находим черный сегмент (самый темный)
    def brightness(color):
        return 0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2]
    
    # Сортируем по яркости, чтобы найти черный
    black_index = np.argmin([brightness(c) for c in segment_colors])
    
    # Поворачиваем список так, чтобы начинался с черного
    segment_colors = segment_colors[black_index:] + segment_colors[:black_index]
    segment_angles = segment_angles[black_index:] + segment_angles[:black_index]
    
    return segment_colors, segment_angles

def display_colors(colors, angles):
    """
    Отображает извлеченные цвета
    """
    print("\n=== СПИСОК ЦВЕТОВ СЕГМЕНТОВ (начиная с черного) ===\n")
    print(f"{'№':<4} {'Угол':<10} {'RGB':<25} {'HEX':<15}")
    print("-" * 60)
    
    for i, (color, angle) in enumerate(zip(colors, angles)):
        r, g, b = int(color[0]), int(color[1]), int(color[2])
        hex_color = f"#{r:02X}{g:02X}{b:02X}"
        rgb_str = f"({r:3d}, {g:3d}, {b:3d})"
        print(f"{i:<4} {angle:<10.1f} {rgb_str:<25} {hex_color:<15}")
    
    # Визуализация
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Показываем цвета как полосу
    color_array = np.array(colors, dtype=np.uint8) / 255.0
    axes[0].imshow([color_array])
    axes[0].axis('off')
    axes[0].set_title('Извлеченные цвета сегментов', fontsize=14, fontweight='bold')
    
    # Показываем отдельные цвета
    n_colors = len(colors)
    for i, color in enumerate(colors):
        axes[1].add_patch(plt.Rectangle((i % 12, i // 12), 0.9, 0.9, 
                                         color=np.array(color)/255.0))
    axes[1].set_xlim(0, 12)
    axes[1].set_ylim(0, 3)
    axes[1].axis('off')
    axes[1].set_title('Отдельные цвета', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

# Использование
if __name__ == "__main__":
    # Замените на путь к вашему изображению
    image_path = 'color_wheel.jpg'  # или 'color_wheel.png'
    
    try:
        colors, angles = extract_color_wheel_colors(image_path)
        display_colors(colors, angles)
        
        # Сохранение в файл
        with open('color_palette.txt', 'w', encoding='utf-8') as f:
            f.write("ЦВЕТОВАЯ ПАЛИТРА (начиная с черного):\n\n")
            for i, (color, angle) in enumerate(zip(colors, angles)):
                r, g, b = int(color[0]), int(color[1]), int(color[2])
                f.write(f"{i}. RGB({r}, {g}, {b}) - HEX: #{r:02X}{g:02X}{b:02X}\n")
        print("\n✓ Цвета сохранены в файл 'color_palette.txt'")
        
    except FileNotFoundError:
        print(f"Ошибка: Файл '{image_path}' не найден!")
        print("Укажите правильный путь к изображению.")