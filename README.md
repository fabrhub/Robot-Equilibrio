# Robot-Equilibrio
Robot che si auto-bilancia per rimanere in equilibrio

#### Lista degli sketch:
| Nome | Libreria PID | Descrizione |
| ------ | ------ | ------ |
| robot_equilibro.ino | QuickPID | prima versione |
| robot_equilibrio_altra_libreria_pid.ino | PID_v1 | prova di altra libreria + migliore calibrazione sensore |
| robot_equilibrio_altra_libreria_pid_autotunePID.ino | PID_v1 + PID_Autotune_Library | Test AutoTune dei valori Kp Ki Kd |
| robot_equilibrio_autotunePID.ino | QuickPID | Test AutoTune dei valori Kp Ki Kd |
| robot_equilibrio_Motor_Deadzone.ino | QuickPID | gestione delle differenze fra i due motori |

>La versione da ritenersi migliore è: **robot_equilibrio_altra_libreria_pid.ino**

![Robot Equilibrio](/img/fronte_wide.jpg "Robot Equilibrio")