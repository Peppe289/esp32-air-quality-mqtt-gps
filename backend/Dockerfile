FROM node:20-alpine3.22

WORKDIR /usr/src/app
RUN apk add --no-cache \
    python3 \
    make \
    g++ \
    build-base

COPY package*.json ./

RUN npm install --production
RUN npm install better-sqlite3

COPY . .

EXPOSE 5000

CMD [ "node", "server.js" ]