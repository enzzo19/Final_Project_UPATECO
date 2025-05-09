"""Controller to drive e-puck to follow the center of the line using camera and OpenCV."""
from controller import Robot
import numpy as np
import cv2

time_step = 32
max_speed = 5
ESCALA = 2  # Factor de escala para agrandar la imagen

robot = Robot()
camera = robot.getDevice('camera')
camera.enable(time_step)
width = camera.getWidth()
height = camera.getHeight()
print(f"Width: {width}, Height: {height}")
nuevo_ancho = int(camera.getWidth() * ESCALA)
nueva_altura = int(camera.getHeight() * ESCALA)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))


def drive_base(left_motor, right_motor, steer, max_speed=5.0):
    # Clampear steer entre -1 y 1
    steer = max(min(steer, 1), -1)

    if steer > 0:
        # Girar hacia la derecha: rueda izquierda más rápida
        left_speed = max_speed
        right_speed = max_speed * (1 - steer)
    else:
        # Girar hacia la izquierda: rueda derecha más rápida
        right_speed = max_speed
        left_speed = max_speed * (1 + steer)

    # Asignar velocidades a motores
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


def process_image(image):
    cv2.imshow("RGB", image)
    """Procesa la imagen para encontrar el centro de la línea negra."""
    # Convertir la imagen a escala de grises
    resized_gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Grises", resized_gray_image)
    # Aplicamos un filtro sobre la imagen y obtenemos una imagen binaria de blanco y negro
    _, binary = cv2.threshold(resized_gray_image, 80, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow("Binaria", binary)
    # Recortamos la parte baja de la imagen que eslo que esta ams cerca de las ruedas y nos interesa
    roi = binary[40:, :] 
    cv2.imshow("ROI", roi)
    # Encontramos los contornos de la línea negra
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Encontrar el contorno más grande (se asume que es la línea)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calcular el momento del contorno
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            # Calcular las coordenadas del centroide
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Dibujar un círculo en el centroide para visualización
            cv2.circle(roi, (cx, cy), 5, (128, 0, 128), -1)
            cv2.imshow("ROI con Centroide", roi)

            # Calcular el error con respecto al centro de la imagen ROI
            center_roi = nuevo_ancho // 2
            print(f"Centro ROI: {center_roi}, cx: {cx}")
            error = cx - center_roi

            # Normalizar el error para obtener el valor de steer (-1 a 1)
            # Ajusta el divisor (por ejemplo, center_roi / 2) para controlar la sensibilidad
            steer = error / (center_roi / 2)
            steer = max(min(steer, 1), -1)  # Asegurar que esté entre -1 y 1
            print(f"Valor de steer: {steer:.2f}")
            return steer
        else:
            return 0.0  # No se pudo calcular el centroide
    else:
        return 0.0  # No se encontraron contornos
    


def run_robot(robot):
    while robot.step(time_step) != -1:
        # Obtener la imagen de la cámara
        image = camera.getImageArray() # Capturar imagenes de la camara
        if image is not None:
            # Convertir la lista de la imagen a un array NumPy
            img_array = np.array(image, dtype=np.uint8)

            # Reformar el array para que tenga las dimensiones correctas (altura, ancho, canales)
            img_bgr = img_array.reshape((camera.getHeight(), camera.getWidth(), 3))
            # Calcular las nuevas dimensiones de la imagen
            dimensiones = (nuevo_ancho, nueva_altura)
            resized_image = cv2.resize(img_bgr, dimensiones, interpolation=cv2.INTER_LINEAR)
            # convert image to rgb 
            resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            
            # Procesar la imagen para obtener el valor de steer
            steer = process_image(resized_image)
            print(f"Valor de steer: {steer:.2f}")

            # Aplicar el valor de steer para controlar el robot
            drive_base(left_motor, right_motor, steer, 5)
            # Mantener las ventanas de OpenCV actualizadas
            cv2.waitKey(1)
    # Quemar todas las ventanas de Opencv al cerrar
    cv2.destroyAllWindows()


if __name__ == '__main__':
    run_robot(robot)