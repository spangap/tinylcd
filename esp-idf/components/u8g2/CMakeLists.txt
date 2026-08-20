# Vendored u8g2 (olikraus/u8g2, csrc/ verbatim at the commit noted in
# csrc/u8g2_fonts_selected.c) — upstream's monolithic u8g2_fonts.c (~38 MB) is
# replaced by that curated subset, and MUI (mui*.c, u8g2's built-in menu
# framework) is dropped: tinylcd has its own page model. INCLUDE_DIRS is public
# on purpose — tinylcd.h hands draw callbacks a u8g2_t* and consumers call the
# u8g2 draw API directly.
file(GLOB U8G2_SRCS "csrc/*.c")
idf_component_register(
    SRCS ${U8G2_SRCS}
    INCLUDE_DIRS "csrc"
)
# Upstream code does not survive the project's -Werror; component-PRIVATE flags
# win over IDF's forced flags (same arrangement as rns' bzip2).
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
