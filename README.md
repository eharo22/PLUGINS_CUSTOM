Este tutorial contempla los pasos para añadir plugins personalizado a Gazebo Sim (Harmonic) 8.9.0.

- REQUISITOS
  Es necesario haber instalado Gazebo Sim (Harmonic) 8.9.0 desde Source tal y como indican los pasos oficiales en:
  https://gazebosim.org/docs/harmonic/install_ubuntu_src/
  Se debe usar el método de vcstool and colcon from pip y durante la construcción del entorno:
   ```
  colcon build --merge-install
   ```
- PASOS
1) Descargar la carpeta del plugin deseado. Verificar que tenga la estructura:
  ```
/PLUGIN1
   |___CMakeLists.txt
   |___include
   |        |___PLUGIN1.hh
   |___src
   |    |___PLUGIN1.cc
  ```
2) Colocar la carpeta del plugin dentro del folder src en la carpeta del workspace generada durante la instalación de Gazebo (/home/<user>/workspace/src/)
3) Activar el entorno de instalación y entra al workspace:
```
. $HOME/vcs_colcon_installation/bin/activate
```
```
cd ~/workspace
```
4) Procede con la reconstrucción de Gazebo, se deberá compilar el plugin:
```
source /home/<user>/workspace/install/setup.bash
colcon build --merge-install
```

5) Tras acabar la compilación, sal del entorno de colcon.
```
deactivate
```

6) Para asegurarte de que fucnionen los plugins es necesario definir el path de los mismos:
```
. ~/workspace/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/workspace/install/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

7) Para que no existan problemas de compilación durante la generaciónn del munod con un nuevo plugin asegúrate de que el SDF posea:
```
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics">
</plugin>
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands">
</plugin>
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster">
</plugin>

```
  Para el caso del plugin WindTests usa estas líneas en el SDF:
```
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <plugin name='gz::sim::systems::AirPressure' filename='gz-sim-air-pressure-system'/>
    <plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'/>
    <plugin name='gz::sim::systems::NavSat' filename='gz-sim-navsat-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
```
8) Para añadir el plugin deseado a un SDF usa:
   - My plugin (impresión de mensaje)
     ```
      <plugin name="my_namespace::MyPlugin" filename="libmy_plugin.so"/>
     ```
   - Table Vertical Force (fuerza vertical)
     ```
     <plugin   filename="libtable_vertical_force.so"      name="gz::sim::v8::systems::TableVerticalForce">
      <step_force>
        <force>50</force>
        <steps>100</steps>
      </step_force>
      <step_force>
        <force>-5</force>
        <steps>800</steps>
      </step_force>
      </plugin>
     ```
      Añade cuantos step_force necesites, recuerda que necesitas siempre un valor de fuerza y de pasos.

   - Wind Tests (fuerza de viento de acuerdo con CSV)
     ```
     <plugin         filename="libwind_tests_plugin.so"      name="gz::sim::v8::systems::WindTests">
        <csv_file>/ruta/al/archivocsv</csv_file>
         <time_per_data_point>1.0</time_per_data_point> <!-- seconds -->
          <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>
     </plugin>

     ```
     Configura el factor de escala y el tiempo de ejecución por par como desees. Recuerda que el archivo
     CSV debe contar solo de 2 columnas sin encabezado con valores de magnitud y dirección (0-360°) de viento.
     
