# Gera components/furuta/furuta_ini.c a partir de pc/furuta.ini antes de cada build (extra_scripts do
# platformio.ini). O CMakeLists.txt do componente também gera, mas só quando o CMake reconfigura, e o PlatformIO
# não reconfigura quando só o .ini muda: sem este script o firmware ficava com o padrão antigo.
# Mesmo formato do CMakeLists.txt; só reescreve se o conteúdo mudou (não força recompilar à toa).
import os

Import("env")  # noqa: F821  (SCons)

_raiz = env.subst("$PROJECT_DIR")  # noqa: F821
_ini = os.path.join(_raiz, "..", "pc", "furuta.ini")
_saida = os.path.join(_raiz, "components", "furuta", "furuta_ini.c")

with open(_ini, "rb") as f:
    _bytes = f.read()
_texto = ("/* gerado pelo CMakeLists.txt a partir de pc/furuta.ini: não editar */\n"
          "const char furuta_ini[] = {" + "".join("0x%02x," % b for b in _bytes) + " 0};\n")
_atual = None
if os.path.exists(_saida):
    with open(_saida, "r", encoding="utf-8", newline="") as f:
        _atual = f.read()
if _atual != _texto:
    with open(_saida, "w", encoding="utf-8", newline="") as f:
        f.write(_texto)
    print("gera_ini: furuta_ini.c atualizado a partir de pc/furuta.ini")
