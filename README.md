# Ros_workshop

## Criar o pacote ros:
Como criar o pacote no ROS e substituir a pasta src por esta, deixem o nome src mesmo OK!
Primeiramente indicamos nossa workspace:
```
$ cd ~/workshop_ws/
$ source devel/setup.bash
```

Criando o pacote dentro da src:
```
$ cd ~/workshop_ws/src
```

Utilizar o catkin_create_pkg para criar o pacote:
```
$ catkin_create_pkg minicurso_package std_msgs rospy roscpp
```
Onde o "minicurso_package" é o nome do nosso pacote e "std_msgs rospy roscpp" são dependencias usadas neste pacote.

![image](https://github.com/HerickDallagnol/Ros_workshop/assets/140270456/a044ff4c-fc1d-4162-9aef-d627b2d1a636)

Compilar o pacote usando o catkin_make: 
```
$ cd ~/workshop_ws/
$ catkin_make
```
Localizamos na src nossos pacote e utilizando o *ls* para ver o que temos dentro da pasta:
```
$ cd ~/workshop_ws/src
$ ls 
```
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/pacotesLS.png)


## Criando scripts
Há diversas maneiras de trabalhar, podemos trabalhar diretamente pelo terminal utilizando ferramentas de editor de texto como o gedit ou podemos utilizar softwares como Sublime Text, Vscode etc.

Dentro do nosso pacote criamos a pasta scripts para manter a organização:
```
$ cd ~/workshop_ws/src/minicurso_package
$ mkdir scripts
$ cd scripts
```
Lembrando que sempre podemos apertar TAB para completar, caso não complete pode ser um *indicativo* de problema. (Geralmente causado pela falta do apontamento com o source/devel)

## Scripts
Vamos criar um publisher para mandar uma variavel e um subscriber para receber esta variavel: (utilizando apenas o terminal)
```
$ cd ~/workshop_ws/src/minicurso_package/scripts
$ gedit publisher.py

```
Codigo do publisher.py:
```
#!/usr/bin/env python3 
import rospy  #biblioteca para o ros
from std_msgs.msg import Int32

#função para enviar a variavel
def enviar_variavel():
    rospy.init_node('var_node', anonymous=True) # iniciar o nodo, aonde 'var_node' é o nome do nosso nodo
    pub = rospy.Publisher('/variavel', Int32, queue_size=10) # Utiliza o rospy.Publisher para declarar nossa publicação, aonde '/variavel' é nosso topico Int32 é o tipo da mensagem 
    variavel = 0
    
    while not rospy.is_shutdown():
        pub.publish(variavel)
        rospy.loginfo("Valor da variavel: %d", variavel)
    
if __name__ == '__main__':
    try:
        enviar_variavel()
    except rospy.ROSInterruptException:
        pass

```
Modificar o *#!/usr/bin/env python3* para determinada versão do python que estiver utilizando.

Utilizamos o chmod +x para dar permissão para o ros executar o codigo.
```
$ chmod +x publisher.py
```
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/chmod.png)
Podemos observar que o arquivo .py ficou verde.

Com o codigo do nosso publisher feito, vamos criar o subscriber para receber nossa variavel:
```
$ cd ~/workshop_ws/src/minicurso_package/scripts
$ gedit subscriber.py
```

Codigo do subscriber.py:
```
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Estou recebendo a variavel %d", data.data) #printa no terminal 
    
#função para iniciar o nodo
def listener():
    rospy.init_node('listener_var_node', anonymous=True) #indica o nodo, aonde 'listener_var_node' é o nome do nosso nodo

    rospy.Subscriber("/variavel", Int32, callback) #se subscreve no nosso topico /variavel, declarada no codigo do publisher 
    rospy.spin() 

if __name__ == '__main__':
    listener()

```
Modificar o *#!/usr/bin/env python3* para determinada versão do python que estiver utilizando.

Utilizamos da mesma maneira o chmod +x para dar permissão para o ros executar o codigo.
```
$ chmod +x subscriber.py
```

Voltamos para o inicio da nossa Workspace e compilamos nosso pacote:
```
$ cd ~/workshop_ws/
$ catkin_make
```
Para rodarmos os nodos, primeiramente iniciamos nosso Master utilizando o *roscore*:
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/roscore.gif)

Utilizamos o rosrun *nome_do_pacote arquivo.py* para rodar nossos nodos separadamente
```
$ rosrun minicurso_package publisher.py
```
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/pub.gif)

Em outro terminal rodamos nosso subscriber da mesma maneira (lembrando que para cada novo terminal temos que direcionar o ROS com a source)
```
$ source devel/setup.bash
$ rosrun minicurso_package subscriber.py
```
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/sub.gif)

Utilizando o *rostopic list* em outro terminal podemos listar os topicos ativos, da mesma forma utilizando o *rosnode list* para listar os nodos. Podemos utlizar o *rostopic echo /nome_do_topico* para ver os seus dados:

![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/rostpiclist.png)

Podemos tambem utilizar o *rqt* para abrir o visualizador gráfico de Nodos/topicos do ROS:
```
$ rqt
```
Caso não tenha instalado:

```
$ sudo apt install ros-noetic-rqt
```

![image](https://github.com/HerickDallagnol/Ros_workshop/assets/140270456/e27741c2-41a5-439d-9894-3b75e1cf32c6)

## Criando um launcher 
Em projetos grandes precisamos rodar diversos nodos, é inviável rodar um por um. Para isso utilizamos um arquivo *.launch* permitindo executar todos os nodos de uma vez e sem a necessidade do Master ativo.

Para isso, entramos no nosso pacote e crimos uma pasta chamada launch para melhor organização:
```
$ cd ~/workshop_ws/src/minicurso_package
$ mkdir launch
$ cd launch
```

Dentro da mesma, criamos nosso arquivo *Sub_pub.launch*:
```
$ gedit Sub_pub.launch

```

Criamos nosso arquivo:
```
<launch>
    <node name="Publisher" pkg="minicurso_package" type="publisher.py" />
    <node name="Subscriber" pkg="minicurso_package" type="subscriber.py" />
</launch>
```

Novamente retornamos para o inicio da nossa Workspace e compilamos nosso pacote:
```
$ cd ~/workshop_ws/
$ catkin_make
```

Para rodar nosso launch utilizamos *roslaunch nome_do_pacote arquivo.launch* :
```
$ roslaunch minicurso_package Sub_pub.launch

```
![](https://raw.githubusercontent.com/HerickDallagnol/Ros_workshop/main/resorces/launch.gif)

