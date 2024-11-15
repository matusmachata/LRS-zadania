# Dokumentácia ku kódom

## Časť 1: Spracovanie mapy a plánovanie trasy (Python kód)

### 1.1 Načítanie PGM súboru (`load_pgm`)
Táto funkcia načíta obrazový súbor vo formáte PGM a overí jeho formát (musí byť vo formáte P2). Načíta rozlíšenie obrazu, maximálnu hodnotu sivej farby a samotné pixely, ktoré vracia ako dvojrozmerné pole numpy a maximálnu sivú hodnotu.
![Vstupná mapa](https://github.com/matusmachata/LRS-zadania/tree/master/img/input_map.png)


### 1.2 Uloženie PGM súboru (`save_pgm`)
Funkcia zapisuje obrázok do PGM súboru vo formáte P2, kde sa každý pixel prepisuje do textovej reprezentácie, aby bol súbor prístupný ďalším programom.

### 1.3 Orezať prázdne okraje (`trim_empty_edges`)
Funkcia odstraňuje prázdne (biele) okraje z obrázka a vracia orezaný obrázok spolu s posunom orezania pre neskoršie korekcie súradníc.

### 1.4 Načítanie bodov trasy (`load_waypoints`)
Načíta súbor CSV obsahujúci body trasy a nastaví stĺpce (`X`, `Y`, `Z`, `Proximity`, `Action`). Výsledkom je tabuľka s uloženými súradnicami a akciami.

### 1.5 Algoritmus kreslenia úsečky (`bresenham_line`)
Implementuje Bresenhamov algoritmus na kreslenie úsečky, ktorý generuje súradnice bodov medzi dvoma bodmi. Je užitočný na kontrolu kolízií a preverenie či je úsek medzi dvoma bodmi priamy a voľný.

### 1.6 Kontrola voľného priameho spojenia (`is_straight_line_clear`)
Funkcia preveruje, či je priamy úsek medzi dvoma bodmi voľný, pomocou `bresenham_line`. Prechádza každým bodom úsečky a kontroluje, či sa v danom bode nenachádza prekážka.

### 1.7 Vyhľadávanie trasy pomocou BFS (`bfs_pathfinding`)
Používa BFS (Breadth-First Search) na hľadanie trasy v blízkosti cieľového bodu vo vzdialenosti `proximity_radius`. Vytvára zoznam bodov pre trasu, ktorá prekonáva prekážky pomocou vyhýbania sa susedným bodom, ak je priamy prístup blokovaný.

### 1.8 Uloženie trasy do textového súboru (`save_all_paths`)
Pre všetky trasy prevádza body z pixelových súradníc späť na metre, pričom každý bod ukladá do textového súboru vo formáte `x, y`. Každá trasa je oddelená prázdnym riadkom.

### 1.9 Hlavná funkcia (`main`)
Obsahuje kroky:
1. Načíta PGM obrázok mapy a orezáva okraje.
2. Načíta body trasy a prevedie ich na pixely s vhodným priblížením.
3. Vytvára mriežku prekážok so zvýšenou bezpečnostnou vzdialenosťou.
4. Vytvára trasy medzi bodmi.
5. Ukladá všetky trasy do textového súboru `all_paths.txt`.
6. Vykreslí trasy na mape a uloží výsledný obrázok.

---

## Časť 2: Riadenie dronu (C++ kód)

### 2.1 Trieda TemplateDroneControl
Obsahuje funkcie pre riadenie dronu pomocou ROS (Robot Operating System) 2. Využíva knižnice mavros_msgs na komunikáciu s dronom, umožňuje sledovanie stavov a príkazov (arming, mód, zmena polohy, vzlet, pristátie) a načítanie waypointov.

### 2.2 Inicializácia ROS uzlov a klientov
V konštruktore triedy sa vytvoria klienti a subscribery na sledovanie stavu dronu (`state_cb`), na odosielanie príkazov na polohu (`local_pos_pub_`) a na odoslanie príkazu na armovanie (`arming_client_`) a na nastavenie režimu dronu (`set_mode_client_`).

### 2.3 Funkcie na načítanie súborov (`readPath`, `readWaypointsCSV`)
1. `readPath` číta súbor s trasami a vytvára zoznam zoznamov bodov, kde každý bod predstavuje cestu medzi waypointmi.
2. `readWaypointsCSV` načítava body trasy (CSV) s atribútmi presnosti a ďalších príkazov.

### 2.4 Transformácia súradníc (`transformPoint`)
Transformuje body z mapy na súradnicový systém dronu, čo umožňuje dronu navigovať na základe bodov z mapy.

### 2.5 Dekódovanie príkazov (`decodeCommand`)
Interpretuje príkazy z CSV súboru pre konkrétne akcie dronu, napríklad “takeoff”, “land” a “yaw180”.

### 2.6 Pohyb dronu (`move`)
Riadi pohyb dronu podľa načítaných trás a waypointov. Funkcia iteruje cez body v trase, aktualizuje pozíciu dronu a sleduje, či sa dron dostal do požadovanej vzdialenosti od cieľa, s možnosťou pristátia alebo vzletu.

### 2.7 Otáčanie dronu (`turn`)
Nastavuje orientáciu dronu podľa zadaného uhla. Používa quaterniony na reprezentáciu otáčania (yaw, pitch, roll) a nastavuje toleranciu pre dosiahnutie presnosti uhla.

### 2.8 Ostatné funkcie
- **`set_mode`**: Nastavuje režim dronu (napríklad “GUIDED”).
- **`arm_drone`**: Armuje dron pre let.
- **`takeoff`**: Riadi vzlet dronu do zadanej výšky.
- **`state_cb`, `local_pos_cb`**: Callbacky na spracovanie stavových správ a aktualizácií polohy dronu.

### 2.9 Hlavná funkcia (`main`)
Spúšťa ROS uzol `TemplateDroneControl` a inicializuje riadenie. Tento uzol čaká na udalosti a spracováva pohyb dronu, kým dron nedosiahne cieľové body.

---

## Zhrnutie
Táto dokumentácia poskytuje prehľad funkcií a štruktúry kódu. Python kód zabezpečuje spracovanie mapy a generovanie trás medzi bodmi, zatiaľ čo C++ kód implementuje algoritmy na ovládanie dronu v ROS prostredí.
"""
