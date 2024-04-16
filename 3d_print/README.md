The idea of the subfolders is:

- `0.1_source_files`: Files directly downloaded from the internet and other then unzipping and minor folder renaming/mergin, they are unmodified
- `0.2_converted`: Parts converted to SolidWorks .sldprt
- `0.3_scaled`: Parts scaled by 5/3, nothing else is done to them
- `1_sldprt`: Current SolidWorks parts with any changes/additions. A kind of source of truth.
- `2_stl`: .stl export for 3D printing
- `3_batches`: 3D printer specific files (.3mf for Bambu, ...) with groups of parts (importted from 2_stl/) for batch printing, indexed in chronologically order

Furthermore, I am also tracking all parts and components in `list_of_parts.xlsx`. 