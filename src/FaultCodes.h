#ifndef FAULTCODES_h
#define FAULTCODES_h

// Returns a human readable description for an Ecodan fault code ("L9", "8000", ...)
const char *faultCodeDescription(const char *code);

// True when the character is valid in a remote controller fault code (0-9, A-Z)
bool isFaultLetter(char value);

#endif
