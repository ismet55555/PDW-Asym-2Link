<h1 align="center">Passive Dynamic Walker (PDW)<br/>Compass Gait (2-Link)<br/>Variable Foot Shape</h1>



A passive dynamic walker (PDW) is a biped robot that does not draw energy from any supply (ie. batteries, gasoline, etc) and only relies on the potential energy of walking down a decline.

This PDW is a so-called compass gait, or 2-link, walking model, in that there are only two effective linkages representing the walker's system.  Essentially, this compass gait model is a inverted double-pendulum rotating about the ground contact.

In addition this PDW model offers a way to change


## :eyeglasses: Overview
* [TODO](#todo)
* [Author](#bust_in_silhouette-author)
* [Licence](#licence)


## :fast_forward: Quick Start
TODO


## :thumbsup: Compatibility
This logger was created and tested on MATLAB R2017b and R2018b on a Windows 10 operating system. Although this logger was created within those software versions and operating systems, it may work in other environments as well.

I would love to hear about usage on earlier or later MATLAB versions and other operating systems.


## :rocket: Installation
1. **Clone this repository to a computer with git.**
    - Use the ol' git machine to clone this repository to to a local directory
    - `git clone https://github.com/ismet55555/Passive-Dynamic-Walker.git`
    - Note that you don't have to use the scary terminal, you can use one of many git graphical interfaces such as [GitHub Desktop](https://desktop.github.com/)
2. **Download this repository as a _.zip_ file.**
    - Go to this repository's front page: https://github.com/ismet55555/Passive-Dynamic-Walker
    - On the right, press the green "Clone or Download" button
    - Select the "Download zip" option on the right.
    - Once downloaded, extract the .zip file on your computer



## Setup
### Simulation Settings
[TODO]

### Initial Conditions
[TODO]

### Slope
[TODO]

### Asymmetry
[TODO]

### Masses
[TODO]

### Mass Positions
[TODO]

### Foot Shape
This model is unique because it is possible to alter the foot shape (or roll over shape) of either foot. The foot shape for this model can be defined as a radius around the walker's ankle as:

<p align="center">radius<sub>foot</sub> = angle*A + B</p>

In this equation _angle_ is the angle around the ankle in radians, _A_ is the change constant (how the foot shape changes), and _B_ is the offset constant.

In addition it is possible to offset the ankle along the top of the foot with a constant _d_ as shown in the figure below.

<p align="center">[Insert Figure Here]</p>

---
## Results
### Animation

### Plots

### File

---
## Parallel Processing
[TODO]




---
## :bust_in_silhouette: Author
**Ismet Handžić** - GitHub: [@ismet55555](https://github.com/ismet55555)

## Reference ##
Different styles
Latex entry (just include file?)

## Licence
This project is licensed under the Apache 2.0 License - Please see the [LICENSE](LICENSE) file for details.
