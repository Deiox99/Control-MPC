# Aspectos importantes a considerar
- El Robot empezara a funcionar siguiendo los siguientes paso:
    1. Prender el interruptor para alimentar los componentes del robot.
    2. Conectarse por medio de SSH a la raspberry.
    3. Una vez dentro de la terminal, buscar la carpeta **Control-mpc** y entrar en ella via **cd**
    4. Activar el entorno virtual llamado **mpc** con el codigo: **source mpc/bin/activate**
    5. ejecutar el comando: **python3 main.py**
    6. Apenas ejecutado el codigo de la raspberry, se debe presionar el boton amarillo conectado a los pines de la ESP-32
    7. Ver el robot funcionar :)

- Los archivos se separan en carpetas, exceptuando el codigo **main.py** y el **esp.ino**:

    1. main.py: Permite ejecutar todos los script para lograr hacer funcionar el robot (En especial el MPC). **Importante:** Puede ejecutar el comando **python3 main.py --sim** para ver el simulador.
    2. /comm/uart_handler.py : Este script permite la comucacion entre la Raspberry y el ESP-32 por medio de UART (RX/TX).
    3. config/robot_params.py : Este script permite definir los parametros del robot, asi como: 
    - radio de las ruedas
    - distancia entre centros
    - los parametros para calcular la trayectoria y el mpc (especificados dentro del archivo)
    4. control/mpc_controller.py : En este script se ejecuta el MPC y entrega los valores que se enviaran a la ESP-32 para calcular el error con el PID.
    5. control_antiguo/control_mpc : codigo de prueba que no se llega a utilizar pero se guarda.
    6. estimation/state_estimator.py : Este script estima la pose del robot (posición X, Y y orientación θ) fusionando los datos del encoder y el giroscopio del MPU6050 mediante un filtro complementario. También incluye la calibración del bias del giroscopio al inicio.
    7. trajectory/lemniscata.py : Este script define la trayectoria de referencia en forma de figura ∞ (Lemniscata de Gerono). Genera los puntos de referencia futuros que el MPC utiliza para calcular los comandos de velocidad. Los parámetros de amplitud y periodo se configuran en **robot_params.py**.
    8. simulation/sim_robot.py : Este script simula el comportamiento del robot diferencial sin necesidad de hardware físico. Permite probar y ajustar el MPC en el PC. Se puede ejecutar de forma independiente con el comando: **python simulation/sim_robot.py**
