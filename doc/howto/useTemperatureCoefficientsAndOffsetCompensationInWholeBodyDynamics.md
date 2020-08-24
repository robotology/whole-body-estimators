To enable the use of temperature compensation and constant offset features in wholebodydynamics device, it is required to add some variables in the config file. The variables can be grouped into three:
- Indicate which sensors belong to the multiple analog sensor category
- Add the temperature coefficients
- Add the offline offset
## Indicating which sensors have multiple analog sensor functionalities

`multipleAnalogSensorsNames` : it is a group that signals which sensors are multipleAnalogSensors ( MAS ).

`SixAxisForceTorqueSensorsNames` : it is a parameter from the multipleAnalogSensorsNames group, that signals which sensors function as six axis force torque sensors.

`TemperatureSensorsNames` : it is a parameter from the multipleAnalogSensorsNames group, that signals which sensors function as temperature sensors.

Example: 
```
<group name="multipleAnalogSensorsNames">
              <param name="TemperatureSensorsNames">(l_leg_ft_sensor,r_leg_ft_sensor,l_foot_ft_sensor,r_foot_ft_sensor)</param>
              <param name="SixAxisForceTorqueSensorsNames">(l_leg_ft_sensor,r_leg_ft_sensor,l_foot_ft_sensor,r_foot_ft_sensor)</param>
</group>`
```
## Add the temperature coefficients
`FT_TEMPERATURE_COEFFICIENTS` : group indicating that the coefficients are to be used for temperature compensation
Inside the group the name of the sensors followed by its coefficients should be indicated as a parameter with the name of the sensor.

Example:

```
       <group name="FT_TEMPERATURE_COEFFICIENTS">
              <param name="l_leg_ft_sensor">(-0.0933  ,  0.2048  ,  1.3342  , -0.0155  ,  0.0027  ,  0.0039  , 30.4064)</param>
        </group>
```

## Add the offline offset
`FT_OFFSET` : group indicating that the offset are to be used when using the `usePreEstimatedOffset` rpc command.
Inside the group the name of the sensors followed by its coefficients should be indicated as a parameter with the name of the sensor.

Example:
```
        <group name="FT_OFFSET">
              <param name="l_leg_ft_sensor">(24.8009  , -6.2369 , -62.0044 , -0.0588 ,  -0.2425  ,  0.1253)</param>
             <!-- <param name="r_leg_ft_sensor">(0.0,0.0,0.0,0.0,0.0,0.0)</param> -->
       </group>
```

