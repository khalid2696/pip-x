# PiP-X: Funnel-based Online Feedback Motion Re-planning

This repository contains: <br />
1. Code for running PiP-X in two environments: maze and forest <br />
2. The data structures and function libraries for executing the code (in `./lib`)  <br /> 
3. A pre-computed library of funnels (in `./funnelLibrary`) <br />

**This codebase is to improve the understanding of the algorithm that appears in [this IJRR paper](https://journals.sagepub.com/doi/abs/10.1177/02783649231209340).**

# Usage
Run `PiPXMazeSense.m` or `PiPXForestSense.m` depending on which workspace you wish to run the algorithm on. <br /> <br />

> Tip: Keep `drawFlag = 1` to get a visual understanding of the workings of the algorithm. Once acquanited with the code, toggle the `drawFlag` to off to quicken the computation speed. 

# Troublehsooting
If you have any questions regarding the codebase or the underlying technical concepts, feel free to reach out to me at khalid26@umd.edu.

# References
If you find this codebase helpful, please consider citing our following works. <br /> <br />

**<ins>Journal version (IJRR 2023)</ins>:** <br />
@article{Jaffar.Otte.IJRR23, <br />
  title={PiP-X: Online feedback motion planning/replanning in dynamic environments using invariant funnels}, <br />
  author={M Jaffar, Mohamed Khalid and Otte, Michael}, <br />
  journal={The International Journal of Robotics Research}, <br />
  pages={02783649231209340}, <br />
  year={2023}, <br />
  publisher={SAGE Publications Sage UK: London, England} } <br /> <br />

**<ins>Book chapter (from WAFR 2022 proceedings)</ins>:** <br />
@inproceedings{Jaffar.Otte.WAFR22, <br />
  title={PiP-X: Funnel-Based Online Feedback Motion Planning/Replanning in Dynamic Environments}, <br />
  author={Jaffar, Mohamed Khalid M and Otte, Michael}, <br />
  booktitle={International Workshop on the Algorithmic Foundations of Robotics}, <br />
  pages={132--148}, <br />
  year={2022}, <br />
  organization={Springer} }
