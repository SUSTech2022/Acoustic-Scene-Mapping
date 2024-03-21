This repository contains versions 1.3 and 2.0 of Multichannel BSS Locate previously available at http://bass-db.gforge.inria.fr/bss_locate/.

# Purpose
Multichannel BSS Locate is a Matlab toolbox to estimate the **directions-of-arrival** (expressed both in azimuth and elevation) of **multiple sources** in a multichannel audio signal recorded by an **array of microphones**. This toolbox relies on the 8 angular spectrum methods provided for 2-channel signals by the [BSS Locate](https://gitlab.inria.fr/bass-db/bss_locate) toolbox.

# Versions
[Version 2.0](v2.0) (September 14, 2018)  
In addition to multichannel multi-source localization, this version provides tools to simulate your own recording scenario (room dimensions, wall absorption coefficients, number of microphones and positions, number of sources and positions, etc.) with an embedded version of the [Roomsimove](http://homepages.loria.fr/evincent/software/Roomsimove_1.4.zip) toolbox, and to evaluate the localization results.

[Version 1.3](v1.3) (November 10, 2016)  
This version was used for the article and the experiments related to the [voiceHome-2 corpus](http://voice-home.gforge.inria.fr/voiceHome-2_corpus.html).

[Saas version](https://allgo.inria.fr/app/multichannelbsslocate)  
This version runs on the A||go platform as a service.


# Usage
Version 2.0: see [documentation](v2.0/doc/Multi-channel%20BSS%20Locate%20User%20Guide.pdf)

Version 1.3: see [documentation](v1.3/doc/Multi-channel%20BSS%20Locate%20User%20Guide.pdf)

# License
Multichannel BSS Locate is derived from [BSS Locate](https://gitlab.inria.fr/bass-db/bss_locate) and [Roomsimove](http://homepages.loria.fr/evincent/software/Roomsimove_1.4.zip). It was authored by Romain Lebarbenchon, Ewen Camberlein, and Nancy Bertin and is released under the terms of the [GNU General Public License (GPL) version 3](https://www.gnu.org/licenses/gpl-3.0.en.html).

The WAV audio files are distributed under the the terms of the [Creative Commons Attribution-NonCommercial-ShareAlike 2.0 license](http://creativecommons.org/licenses/by-nc-sa/2.0/).

# Support
For any suggestions or questions about the toolbox, please contact [support.mbss.locate@inria.fr](mailto:support.mbss.locate@inria.fr).