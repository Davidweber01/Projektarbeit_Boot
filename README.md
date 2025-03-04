# **Projektarbeit_Boot GPS Tracking & Visualisierung**

Für das Prototyping werden Eval-Boards verwendet bis zum finalen Test. Anschließend wird ein eingenes PCB erstellt. Mögliche Boards sind STM32Fx oder auch MSP32 von TI.
## 1. GPS Modul

Für die GPS Daten wird ein GPS Modul verwendet, Johannes Faller wird eins bereitstellen, andernfalls wird das Modul _Neo 6M_ verwendet. Tutorial wie man das Modul mit einem STM32 ansteuern könnte gibt es auch.

[Tutorial für GPS Ansteuerung mit STM32](https://controllerstech.com/gps-neo-6m-with-stm32/)

Die Daten werden über UART vom Modul ausgelesen, sie befinden sich noch _NMEA_-Format und müssen dann noch decodiert werden.

## 2. LORA Modul

Die Darstellung der GPS-Daten soll webbasiert erfolgen, dementsprechend müssen die Daten hochgeladen werden. Für diese Funktion wird _LoRa_ verwendet, ein drahtloser Übertragungskanal, der für IOT-Appliaktion verwednet wird.
[Näheres zu LoRa findet man hier.](https://www.thethingsnetwork.org/docs/lorawan/what-is-lorawan/)

## 3. PCB-Design

Sobald das GPS-Tracking und die Visualisierung funktioniert, kann das PCB erstellt werden. Verwendet wird das Tool _KiCad_. Nach Absprache mit den Betreuern wird das PCB dann bestellt mit den Bauteilen und dann gelötet. 

## 4. Gehäuse und Stromversorgung

Ist das PCB fertig designt , so kennt man nun die Maße und kann das Gehäuse entwickeln. Es soll möglichst kompakt und vor allem wasserdicht sein. Zudem soll man Device ladne können, also muss auch an einen kleinen Akku samt Ladebuchse fürs Gehäuse gedacht werden.
Das Gehäuse kann an der HTWG 3D gedruckt werden, dafür benötigen wir ein CAD-Modell.

_Als kleine Nebenaufgabe/Nebeneigenschaft wäre ein kleines Solarpanel am Gehäuse zu montieren, damit das Device für eine Weile autar laufen könnte (natürlich wasserdicht)._

## 5. End device Info TTN
AppEUI: 583875E9A9FE9FA9
DevEUI: 70B3D57ED8003D8F

Device Address: 27FC40EE
