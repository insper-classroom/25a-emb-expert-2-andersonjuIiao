## HID

Bluetooth HID protocol example from bltstack.

- hid_keyboard_demo.c: A pico se passa por um teclado bluetooth, ao conectar um computador nela é possível emular um teclado.
- hid_mouse_demo.c:  A pico se passa por um mouse bluetooth, ao conectar um computador nela é possível emular um mouse.

No exemplo do keyboard você deve abrir o terminal e enviar uma string, a pico vai emular que está apertando essas teclas.

## Selecionando o exemplo

Para selecionar qual exemplo quer executar, modique o `/hid/CMakeList.txt`:

- keyboard

``` diff
add_executable(pico_emb
+        hid_keyboard_demo.c
        main.c
)
```

- mouse

``` diff
add_executable(pico_emb
+        hid_mouse_demo.c
        main.c
)
```


