# TFG
codigo fuente y memoria del puto TFG

------------------------------------------------------------------ESTADO---------------------------------------------------------------

memoria: Todo bien mientras siga echándole horitas  
código:  
visualizer: Funciona bien si le meto .pcd, nada fancy con que funcione vale  
sift keypoints: Extracción de keypoints sift basándose en la estiamción de la normal a la superficie, ver que coño esncada parámetro
sift_keypoints_estimation: sift keypoints a pelo, error de acceso de memoria  
sift_z_keypoints_estimation: sift keypoints con coordenada z, error de acceso de memoria   
harris keypoints: harris keypoints, error ni idea  
narf keypoints: narf keypoints, error de creación de range image/far ranges  
points clouds: nubes de puntos en formato .pcd  


cros compilación: error de llamada a metodos virtuales de PCDWrite
compilación directa: todo guay con el swap file de 2GB
          


------------------------------------------------------------------OBJETIVOS------------------------------------------------------------

1.- Implementar en software (en PC) un sistema básico de visión 3D para registrar nubes de puntos. Como verás en [1], el registro de una nube de puntos consiste en combinar varias capturas de una cámara en una única nube, viendo cómo alinearlas correctamente. 
•	Dentro del algoritmo, me gustaría empezar por la detección de keypoints (como SIFT). 
•	Estaría bien que además puedas mostrar el resultado del registro con las opciones de PCL. 
Gran parte de este trabajo está ya montado en PCL como ejemplos en los tutoriales. Se trata de probarlo y entender bien cómo funciona. Te mando un artículo y una presentación que espero que te puedan ayudar.
Proceso: detectar keypoints – alienamiento inicial – filtrado – alineamiento iterativo – llegar a resultado según criterio de iteraciones, transformación…
2.- Compilar la misma aplicación para una Pynq, que es una FPGA que lleva un micro ARM dentro, en el que tenemos instalado Linux. 
•	Sobre el algoritmo funcionando en los ARMs haremos un profiling a ver cómo funciona, ver el algoritmo que lleva más tiempo en su ejecución, etc...

3.- De los bloques que lleven más tiempo, seleccionaremos uno (como el de keypoint detecto, por ejemplo) para llevar a hardware en VHDL ( o con un lenguaje de síntesis de alto nivel como HLS, ya te diré como). Esta parte dependerá del tiempo que te lleve lo anterior.


He puesto los objetivos de manera incremental, así que creo que lo mejor es que empieces por implementar lo del registro de nubes de punto sobre PCL en tu PC. Para ello, empieza por leer los documentos que te mando y en cuanto los tengas e intentes compilar una primera aplicación como la detección de keypoints, pásate por el laboratorio para ver cómo vas y resolver dudas. 
Después, para la parte de la FPGA, tendrás que venir unos días seguidos para que te pueda echar una mano, porque tienes que ponerte con Vivado (el sustituto de ISE en Xilinx)  De todas formas, si te quieres pasar antes, miércoles y jueves estoy libre. Y si en algún momento te ves parado, avísame para que podamos verlo en el laboratorio.

                     ___..-.---.---.--..___
               _..-- `.`.   `.  `.  `.      --.._
              /    ___________\   \   \______    \
              |   |.-----------`.  `.  `.---.|   |
              |`. |'  \`.        \   \   \  '|   |
              |`. |'   \ `-._     `.  `.  `.'|   |
             /|   |'    `-._o)\  /(o\   \   \|   |\
           .' |   |'  `.     .'  '.  `.  `.  `.  | `.
          /  .|   |'    `.  (_.==._)   \   \   \ |.  \         _.--.
        .' .' |   |'      _.-======-._  `.  `.  `. `. `.    _.-_.-'\\
       /  /   |   |'    .'   |_||_|   `.  \   \   \  \  \ .'_.'     ||
      / .'    |`. |'   /_.-'========`-._\  `.  `-._`._`. \(.__      :|
     ( '      |`. |'.______________________.'\      _.) ` )`-._`-._/ /
      \\      |   '.------------------------.'`-._-'    //     `-._.'
      _\\_    \    | AMIGA  O O O O * * `.`.|    '     //
     (_  _)    '-._|________________________|_.-'|   _//_
     /  /      /`-._      |`-._     / /      /   |  (_  _)
   .'   \     |`-._ `-._   `-._`-._/ /      /    |    \  \
  /      `.   |    `-._ `-._   `-._|/      /     |    /   `.
 /  / / /. )  |  `-._  `-._ `-._          /     /   .'      \
| | | \ \|/   |  `-._`-._  `-._ `-._     /     /.  ( .\ \ \  \
 \ \ \ \/     |  `-._`-._`-._  `-._ `-._/     /  \  \|/ / | | |
  `.\_\/       `-._  `-._`-._`-._  `-._/|    /|   \   \/ / / /
              /    `-._  `-._`-._`-._  ||   / |    \   \/_/.'
            .'         `-._  `-._`-._  ||  /  |     \
           /           / . `-._  `-._  || /   |      \
          '\          / /      `-._    ||/'._.'       \
           \`.      .' /           `-._|/              \
            `.`-._.' .'               \               .'
              `-.__\/                 `\            .' '
                                       \`.       _.' .'
                                        `.`-._.-' _.'
                                          `-.__.-'


