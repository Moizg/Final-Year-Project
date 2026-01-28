# Optimizing Traffic Scheduling using Game Theory

An intelligent traffic signal controller that replaces fixed-time cycles with a Game Theory algorithm to dynamically optimize traffic flow in chaotic, mixed-traffic environments.

## Description

This project addresses the inefficiencies of static traffic signals in urban areas with high unpredictability (e.g., Pakistan). By modeling the intersection as a non-cooperative game, the system treats road lanes as "players" competing for Green time. The controller calculates a Nash Equilibrium-like state in real-time based on queue lengths, minimizing total system wait time.

This version **1.0** release (The "Slipstream" Version) features a custom-designed 4-way intersection with dedicated **Slip Lanes** (Left-Turn bypasses) to separate turning traffic from the main signal flow, further enhancing throughput. The simulation includes realistic mixed traffic: rickshaws, motorcycles, cars, and heavy vehicles with aggressive driving behaviors (lane weaving, gap acceptance).

## Getting Started

### Dependencies

* **SUMO (Simulation of Urban MObility):** Version 1.10.0 or higher.
* **Python:** Version 3.7 or higher.
* **Operating System:** Windows 10/11, Linux, or macOS.
* **Environment Variable:** You must have `SUMO_HOME` set in your system environment variables.

### Installing

1.  **Download SUMO:**
    Install SUMO from the [official Eclipse website](https://www.eclipse.org/sumo/).

2.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/Moizg/Final-Year-Project.git](https://github.com/Moizg/Final-Year-Project.git)
    cd Final-Year-Project
    ```

3.  **Verify Environment:**
    Ensure your `SUMO_HOME` variable is correctly pointing to your SUMO installation directory (e.g., `C:\Program Files (x86)\Eclipse\Sumo`).

### Executing program

To run the simulation with the Game Theory controller:

1.  Open your terminal or command prompt in the project folder.
2.  Run the Python controller script. This will automatically launch the SUMO GUI.

```bash
python controller.py
```
3. In the SUMO GUI window that appears:

  * Adjust the "Delay" slider (top toolbar) to ~100ms to visualize the traffic flow.
  * Press the green "Play" button.

## Help

Common Issue: "Error: tcpip::Socket::recvAndCheck @ recv: peer shutdown" This usually happens if the simulation crashes immediately.

* Check if the .net.xml file matches the detector definitions in .add.xml.

* Try running the simulation without Python first to see the specific error:
  ```bash
  sumo-gui -c run.sumocfg
  ```

## Authors

[Abdul Moiz Ghazanfar] - @Moizg

[Subhan Bokhari]

[Maaz Mustafa]

## Version History

* 1.0
    * Initial Release: "Slipstream" Version.
    * Features: Queue-length based simple Payoff Function, Slip Lane Geometry, Heterogeneous Traffic Model.

## License

This project is licensed under the [MIT] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* Prof. Ben Polak (Yale University) - For the foundational Game Theory concepts.
* Eclipse SUMO Community - For documentation and tooling support.
* [README Template] - (https://gist.github.com/DomPizzie/7a5ff55ffa9081f2de27c315f5018afc)
