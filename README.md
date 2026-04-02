This project uses feedforward and feedback controllers to separate tracking and stabilization capabilities. It can promise there will be non-overshoot behaviour for all plants in matlab simulation.

#### Topology

![](./media/layout.jpg)

#### Minimum example 

Add "./Function" folder to path

```matlab
plant_num = [2,10];
plant_denum = [1,1,0,0];
plant = tf(plant_num,plant_denum);
stabilization_controller = TwoDOFFeedbackTuningFunction(Plant);
tracking_controller = TwoDOFtuningFunction(plant_num,plant_denum,stabilization_controller);
```

#### Example output

<img src="./media/non-overshoot.jpg" style="zoom:50%;" />

#### Demo - Quanser Interactive Lab

[demo vedio](./media/quanser_vedio.mp4) - Video under "./media/quanser_vedio.mp4" folder

