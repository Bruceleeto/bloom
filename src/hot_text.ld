/* Linker section ordering (ld --section-ordering-file): keep these hot functions together.
 * KOS's startup code stays first so the image still starts at 0x8c010000. */
.text : {
  *_kos_startup.o(.text)
  *(.text.gpuchain)
}
