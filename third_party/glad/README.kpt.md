# KPT GLAD provenance

These files were generated with GLAD 2.0.8 at upstream commit
`73db193f853e2ee079bf3ca8a64aa2eaf6459043`.

Generator inputs:

- API/profile: `gl:core=3.3`
- extensions: none
- OpenGL registry commit:
  `9d527dbc81bb76e35ba284fe385ed8a5ddb90cbc`
- EGL registry commit (source of `KHR/khrplatform.h`):
  `3d7796b3721d93976b6bfe536aa97bbc4bce8667`

Generation command:

```bash
python -m glad --api gl:core=3.3 --extensions '' --out-path generated c
```

Normal builds use the checked-in output and neither download nor regenerate it.
Output SHA-256 values:

```text
90801584c2a28033c156ca418481934a6060478120b7b7283fe9d2ef09295d7c  include/glad/gl.h
7b1e01aaa7ad8f6fc34b5c7bdf79ebf5189bb09e2c4d2e79fc5d350623d11e83  include/KHR/khrplatform.h
421def813ec71345297c1a707a738cb2b01358ef8bf849f64ba45fad66277466  src/gl.c
```
