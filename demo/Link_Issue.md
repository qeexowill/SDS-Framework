# Issue linking with CMSIS-DSP functions

This branch of the SDS-Framework demo has a minor change which illustrates a problem linking applications.
To reproduce, follow the directions in [README.md](./README.md), essentially:
1. Install Packs:
   ```
   csolution list packs -s Demo.csolution.yml -m >packs.txt
   cpackget add -f packs.txt
   ```
2. Use the `csolution` command to create `.cprj` project files.
   ```
   > csolution convert -s Demo.csolution.yml
   ```

3. Use the `cbuild` command to create executable files.
   ```
   > cbuild Demo.Debug+AVH.cprj
   ```

The build command should end with an error like:
```
> Error: L6218E: Undefined symbol arm_cfft_sR_f32_len16 (referred from demo.o).
```

This symbol comes from Line 147 in [demo.c](./demo.c). From reading the documentation on CMSIS-DSP, I get the sense that I need to configure the library
to compile in certain tables, but with command line `cpackget`/`csolution`/`cbuild`, I can't figure out how to do that. 
