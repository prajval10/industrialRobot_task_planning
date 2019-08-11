# Implementation of Human-Robot Collaboration Models in Manifacturing Scenarios

Master's Thesis (M.S) for Robotics Engineering ([EMARO](https://master-emaro.ec-nantes.fr/)) program at the [University of Genova, Italy](https://unige.it/).

**Supervisors**: [Prof. Fulvio Mastrogiovanni](https://www.dibris.unige.it/mastrogiovanni-fulvio) and [Dr. Kourosh Darvish](https://www.iit.it/people/kourosh-darvish)

<p align="center">
  <img src="https://user-images.githubusercontent.com/22452731/62788941-38c26400-bac8-11e9-8e76-ef5eb51a4cb7.gif" />
</p>

### Software Architecture
The overall cooperation task is based on a modular, reactive architecture. The  architecture  has  two  phases,  namely  the Offline  Phase and  the Online  Phase.  The Offline  Phase is  composed  of teaching  safe  way-points  for  the  robot  in  the  manufacturing work-cell.   This   process   is   application-dependent   and   is done  by  a  robot  programmer.

The Online Phase has three layers namely the representation layer shown in blue,the perception layer in orange and interface layer in green. 
- Representation layer: Responsible for task representation, task allocation and task planning. 
- Perception layer: Responsible to find the object pose and grasp location to the robot.
- Interface layer: Responsible for providing interfaces to robot (robot drivers) and humans (GUIs).

<p align="center">
  <img src="https://user-images.githubusercontent.com/22452731/62839986-78897700-bc93-11e9-8ddd-9ad00ae4ad15.png" width="600" height="700" />
</p>
