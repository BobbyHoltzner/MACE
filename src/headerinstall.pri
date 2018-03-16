for(header, INSTALL_HEADERS) {
  path = $${INSTALL_PREFIX}/$${dirname(header)}
  eval(export.files += $$header)
  eval(export.path = $$path)
  eval(INSTALLS *= export)
}
