// Reinício no modo de gravação sem os botões (gravacao.cpp).
#pragma once

void gravacao_init();             // depois do device.begin(): o toque de 1200 baud na porta CDC reinicia para gravar
void gravacao_reinicia();         // reinicia já no modo de gravação do ROM (USB-Serial-JTAG)
void gravacao_reinicia_depois();  // idem em 50 ms (de dentro de um comando, para a resposta sair)
