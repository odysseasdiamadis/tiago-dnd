# Intro and goal of the project
The aim of this project is to reproduce a Dungeons and Dragons session with Tiago as a Dungeon Master.

# Requirements (to update)
pip install gtts
sudo apt update
sudo apt install mpg123


## How to start the simulation
run start_docker.sh, then launch the world with the command in commands.md.
To work or send ros commands on Tiago, run join.sh in another terminal window. 

## TODOS
- [ ] Riconoscimento giocatori
    - [ ] Avere un set predefinito di posizioni, e tiago chiede uno ad uno come si chiama o di indentificarsi
- [ ] tiago deve poter indicare i giocatori per farli giocare a turni

### Funzionalità
- [ ] Memorizzare la posizione dei giocatori
- [ ] Indicare i giocatori
    - [ ] muovere testa e braccio nella loro direzione (ROS)
- [ ] parlare ai giocatori (TTS)
- [ ] capire cosa dicono i giocatori e tenere uno storico delle loro conversazioni (STT)
    - [ ] capire quando uno smette di parlare
