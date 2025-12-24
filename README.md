# ROS2 Turtlesim – Spawn & Go To Location Project

Bu proje, ROS2 ve `turtlesim` kullanılarak geliştirilen,  
**rastgele kaplumbağa spawn eden**,  
**bu kaplumbağaları custom message ile yayınlayan** ve  
**ana kaplumbağayı hedefe yönlendirip servis ile yakalayan**  
uçtan uca bir ROS2 uygulamasıdır.

Proje; **topic, service, custom msg/srv ve kontrol algoritması** kavramlarını
birlikte göstermek amacıyla hazırlanmıştır.

---

## 📁 Proje Yapısı

```text
turtlesim_ws/
└── src/
    ├── turtlesim_interfaces/
    │   ├── msg/
    │   │   ├── Turtle.msg
    │   │   └── TurtleArray.msg
    │   ├── srv/
    │   │   └── CatchTurtle.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── turtlesim_py_pkg/
        ├── turtlesim_py_pkg/
        │   ├── spawn_turtle.py
        │   └── go_to_location.py
        ├── package.xml
        ├── setup.py
        └── resource/
