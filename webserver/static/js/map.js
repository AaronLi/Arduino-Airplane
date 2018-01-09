function initMap() {
  var uluru = {lat: 42.24021040494719, lng: -82.98437118530273};
  var map = new google.maps.Map(document.getElementById('map'), {
    zoom: 16,
    center: uluru,
    clickableIcons: false,
    streetViewControl: false,
    mapTypeId: "hybrid",
    mapTypeControl: false
  });
  var marker = new google.maps.Marker({
    position: uluru,
    map: map
  });
  google.maps.event.addListener(map, 'click', function(e){
    console.log('Position:', e.latLng.toString());
    comSock.send(e.latLng.toString());
    var marker = new google.maps.Marker({
      position: e.latLng,
      map: map
    });
  })
}
